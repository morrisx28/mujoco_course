"""Given MuJoCo adapter. No controller or student QP answers are imported here."""
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from scipy.optimize import lsq_linear
ROOT = Path(__file__).resolve().parents[1]

class Duck:
    def __init__(self, kp=60., kd=3.):
        self.model = mujoco.MjModel.from_xml_path(str(ROOT/'robot/openduck_mini_v2/scene_flat_terrain.xml'))
        m=self.model
        m.opt.timestep=.002; m.opt.iterations=60; m.opt.ls_iterations=20
        m.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        m.actuator_gainprm[:,0]=kp; m.actuator_biasprm[:,1]=-kp; m.actuator_biasprm[:,2]=-kd
        # Keep original force/joint limits and friction; reduce redundant passive damping.
        m.dof_damping[6:]=.1
        self.data=mujoco.MjData(m); self.ikdata=mujoco.MjData(m)
        self.feet=np.array([mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,s+'_foot') for s in ['left','right']])
        self.footgeoms=np.array([mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,s+'_foot_bottom_tpu') for s in ['left','right']])
        self.floor=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'floor')
        # The source model only enables sole collisions. Enable non-foot visual
        # meshes against the floor only, so the non-foot-ground check is meaningful.
        foot_bodies=set(m.geom_bodyid[self.footgeoms])
        m.geom_contype[self.floor]=3
        for geom in range(m.ngeom):
            if geom!=self.floor and m.geom_bodyid[geom] not in foot_bodies:
                m.geom_contype[geom]=0; m.geom_conaffinity[geom]=2

        self.joints=m.actuator_trnid[:,0].copy();self.qadr=m.jnt_qposadr[self.joints];self.vadr=m.jnt_dofadr[self.joints]
        self.names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_JOINT,int(j)) for j in self.joints]
        self.low=m.jnt_range[self.joints,0]+.001;self.high=m.jnt_range[self.joints,1]-.001
        mujoco.mj_resetDataKeyframe(m,self.data,0);mujoco.mj_forward(m,self.data)
        self.qcommand=self.data.qpos[self.qadr].copy()

    def state(self):
        m,d=self.model,self.data
        mujoco.mj_forward(m,d)
        jac=np.zeros((3,m.nv));mujoco.mj_jacSubtreeCom(m,d,jac,1)
        return d.subtree_com[1].copy(),jac@d.qvel

    def ik(self, com, feet, seed=None, iterations=12, trunk_pitch=-.05236):
        m,d=self.model,self.ikdata
        d.qpos[:]=self.data.qpos if seed is None else seed
        d.qvel[:]=0
        d.qpos[self.qadr]=np.clip(d.qpos[self.qadr],self.low,self.high)
        targetR=Rotation.from_euler('y',trunk_pitch).as_matrix()
        for _ in range(iterations):
            mujoco.mj_forward(m,d)
            rows=[]; errors=[]
            for k,site in enumerate(self.feet):
                jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacSite(m,d,jp,jr,int(site))
                rows.extend([30*jp,2*jr[1:]])
                errors.extend([30*(feet[k]-d.site_xpos[site]),2*Rotation.from_matrix(d.site_xmat[site].reshape(3,3).T).as_rotvec()[1:]])
            jc=np.zeros((3,m.nv));mujoco.mj_jacSubtreeCom(m,d,jc,1)
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);mujoco.mj_jacBody(m,d,jp,jr,1)
            rows.extend([30*jc,3*jr]);errors.extend([30*(com-d.subtree_com[1]),3*Rotation.from_matrix(targetR@d.xmat[1].reshape(3,3).T).as_rotvec()])
            # Head/neck remain neutral; retain the valid knee branch through warm starts.
            posture=np.zeros((4,m.nv));posture[np.arange(4),self.vadr[5:9]]=1
            rows.append(2*posture);errors.append(-2*d.qpos[self.qadr[5:9]])
            J=np.vstack(rows); e=np.concatenate(errors)
            J=np.vstack([J,.02*np.eye(m.nv)]);e=np.r_[e,np.zeros(m.nv)]
            lower=np.full(m.nv,-.10);upper=-lower
            lower[self.vadr]=np.maximum(lower[self.vadr],self.low-d.qpos[self.qadr])
            upper[self.vadr]=np.minimum(upper[self.vadr],self.high-d.qpos[self.qadr])
            delta=np.linalg.lstsq(J,e,rcond=None)[0]
            if np.any(delta<lower) or np.any(delta>upper):
                delta=lsq_linear(J,e,bounds=(lower,upper),tol=1e-7,max_iter=60).x
            mujoco.mj_integratePos(m,d.qpos,delta,1.)
            if np.max(np.abs(delta))<1e-6:break
        mujoco.mj_forward(m,d)
        error=max(np.max(np.linalg.norm(feet-d.site_xpos[self.feet],axis=1)),np.linalg.norm(com-d.subtree_com[1]))
        return d.qpos.copy(),float(error)

    def initialize(self,com,feet):
        q,err=self.ik(com,feet,iterations=100)
        if err>.01:raise RuntimeError(f'Initial IK error {err:.4f} m')
        self.data.qpos[:]=q;self.data.qvel[:]=0;self.qcommand=q[self.qadr].copy()
        mujoco.mj_forward(self.model,self.data)
        return err

    def step(self,target):
        # A 12 rad/s command slew limit, in addition to hard position/force limits.
        self.qcommand+=np.clip(np.asarray(target)-self.qcommand,-12*.002,12*.002)
        self.data.ctrl[:]=np.clip(self.qcommand,self.low,self.high)
        mujoco.mj_step(self.model,self.data)

    def contacts(self):
        contact=np.zeros(2,dtype=bool);bad=False
        for index,c in enumerate(self.data.contact):
            force=np.zeros(6);mujoco.mj_contactForce(self.model,self.data,index,force)
            if force[0]<.2:continue
            if self.floor not in (c.geom1,c.geom2):continue
            other=c.geom2 if c.geom1==self.floor else c.geom1
            hits=np.where(self.footgeoms==other)[0]
            if len(hits):contact[hits[0]]=True
            else:bad=True
        return contact,bad
