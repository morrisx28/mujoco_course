"""Given finite reference planner and course-format adapter.

Uses the Lecture 5 Placo engine's task-space planner, then the MuJoCo-model IK
adapter selects a joint-limit-compliant branch. No teacher controller is imported.
"""
from dataclasses import dataclass
from pathlib import Path
import sys,json
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'lecture5'))

@dataclass
class Reference:
    time: np.ndarray
    com: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    feet: np.ndarray
    contacts: np.ndarray
    zmp: np.ndarray
    height: float
    end_walk: float
    joint_names: tuple=()
    joints: object=None

    def index(self,t):
        return min(int(np.floor((max(0,t)+1e-9)/.01)),len(self.time)-1)

    def preview(self,t,N=50,dt=.02,anchor=None):
        anchor=np.zeros(2) if anchor is None else np.asarray(anchor)
        ids=np.array([self.index(t+k*dt) for k in range(N+1)])
        states=np.c_[self.com[ids,:2]+anchor,self.velocity[ids,:2]]
        polys=[];corners=np.array([[-.025,-.016],[.025,-.016],[.025,.016],[-.025,.016]])
        for i in ids[:-1]:
            polys.append(np.concatenate([self.feet[i,k,:2]+anchor+corners for k in np.flatnonzero(self.contacts[i])]))
        return states,self.zmp[ids[:-1],:2]+anchor,polys


def generate_reference(steps=24,height=.22,step_length=.012,single_support=.35,foot_height=.015):
    import placo
    from openduck.engine import PlacoWalkEngine,_make_supports
    from openduck.reference import ROBOT_DIR
    params=json.loads((ROBOT_DIR/'placo_defaults.json').read_text())
    params.update(walk_com_height=height,walk_foot_height=foot_height,single_support_duration=single_support)
    engine=PlacoWalkEngine(ROBOT_DIR/'open_duck_mini_v2.urdf',params,(step_length,0.,0.))
    engine.planner.configure(step_length,0.,0.,steps)
    feet=engine.planner.plan(placo.HumanoidRobot_Side.left,
                           placo.flatten_on_floor(engine.robot.get_T_world_left()),
                           placo.flatten_on_floor(engine.robot.get_T_world_right()))
    supports=_make_supports(feet,engine.parameters.has_double_support())
    tr=engine.walk.plan(supports,engine.robot.com_world(),0.)
    times=np.arange(round((tr.t_end+1.)/.01)+1)*.01
    c=[];v=[];a=[];f=[];contacts=[];p=[]
    for t in times:
        tau=min(float(t),tr.t_end-1e-8)
        c.append(tr.get_p_world_CoM(tau));v.append(tr.get_v_world_CoM(tau) if t<tr.t_end else np.zeros(3))
        a.append(tr.get_a_world_CoM(tau) if t<tr.t_end else np.zeros(3))
        feet=np.array([tr.get_T_world_left(tau)[:3,3],tr.get_T_world_right(tau)[:3,3]])
        feet[:,2]+=.002 # Site target above the ground; physical mesh contacts remain active.
        f.append(feet)
        if tr.support_is_both(tau):contacts.append([1,1])
        elif tr.support_side(tau)==placo.HumanoidRobot_Side.left:contacts.append([1,0])
        else:contacts.append([0,1])
        p.append(tr.get_p_world_ZMP(tau,np.sqrt(9.81/height))[:2])
    return Reference(times,np.array(c),np.array(v),np.array(a),np.array(f),np.array(contacts),np.array(p),height,float(tr.t_end))



def export_reference(ref,robot,path):
    """Given course-v1 serializer; joint/frame/velocity mathematics was taught in HW5."""
    from openduck.motion import FIELDS,CONVENTIONS,save_episode
    root_pos=[];quat=[];joint=[];toes=[];seed=robot.data.qpos.copy()
    for c,f in zip(ref.com,ref.feet):
        seed,error=robot.ik(c,f,seed=seed,iterations=12)
        if error>.01:raise RuntimeError(f'Reference IK error {error:g}')
        R=Rotation.from_quat(seed[3:7][[1,2,3,0]])
        root_pos.append(seed[:3].copy());quat.append(R.as_quat());joint.append(np.r_[seed[robot.qadr],0,0])
        toes.append((R.as_matrix().T@(f-seed[:3]).T).T)
    root_pos,quat,joint,toes=map(np.asarray,(root_pos,quat,joint,toes))
    for k in range(1,len(quat)):
        if quat[k]@quat[k-1]<0:quat[k]*=-1
    angular=np.array([(Rotation.from_quat(quat[k])*Rotation.from_quat(quat[k-1]).inv()).as_rotvec()/.01 for k in range(1,len(quat))]);angular=np.vstack([angular[0],angular])
    names=robot.names+['left_antenna','right_antenna'];sizes=[3,4,len(names),3,3,3,3,len(names),3,3,2]
    vel=lambda x:np.gradient(x,.01,axis=0,edge_order=2)
    frames=np.c_[root_pos,quat,joint,toes[:,0],toes[:,1],vel(root_pos),angular,vel(joint),vel(toes[:,0]),vel(toes[:,1]),ref.contacts]
    offsets=np.r_[0,np.cumsum(sizes)[:-1]]
    episode=dict(FormatVersion='openduck-course-1',Robot='open_duck_mini_v2',Joints=names,
                 CoordinateConventions=CONVENTIONS,Frame_offset=[dict(zip(FIELDS,map(int,offsets)))],Frame_size=[dict(zip(FIELDS,sizes))],
                 Frames=frames.tolist(),Times=ref.time.tolist(),FPS=100,FrameDuration=.01,LoopMode='Clamp',CycleOffsetPosition=False,CycleOffsetRotation=False,
                 MotionWeight=1.,Vel_x=float(vel(root_pos)[:,0].mean()),Vel_y=float(vel(root_pos)[:,1].mean()),Yaw=float(angular[:,2].mean()))
    save_episode(episode,path)
    np.savez_compressed(Path(path).with_suffix('.tasks.npz'),time=ref.time,com=ref.com,velocity=ref.velocity,acceleration=ref.acceleration,feet=ref.feet,contacts=ref.contacts,zmp=ref.zmp,height=ref.height,end_walk=ref.end_walk)
    return episode


def interpolate_episode(episode,t):
    """Named-layout reader with xyzw-to-wxyz conversion and quaternion SLERP."""
    from openduck.motion import validate_episode,frame_slices
    validate_episode(episode);times=np.asarray(episode['Times']);frames=np.asarray(episode['Frames']);sl=frame_slices(episode)
    t=float(np.clip(t,times[0],times[-1]));j=min(np.searchsorted(times,t,side='right')-1,len(times)-2);j=max(j,0)
    u=(t-times[j])/(times[j+1]-times[j]);row=(1-u)*frames[j]+u*frames[j+1]
    q=Slerp(times[j:j+2],Rotation.from_quat(frames[j:j+2,sl['root_quat']]))([t]).as_quat()[0]
    return dict(root_pos=row[sl['root_pos']],root_quat_wxyz=q[[3,0,1,2]],joints=row[sl['joints_pos']],names=episode['Joints'])
