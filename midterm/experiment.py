"""Given reproducible physics runner, diagnostics and plots.

Controller callback: (state, reference_states, zmp_reference, polygons, previous,
height, dt) -> (first_zmp, predicted_states, QPResult).
"""
from pathlib import Path
import json,time
import numpy as np
import matplotlib.pyplot as plt
import mujoco
from robot import Duck


def run_experiment(ref,controller=None,mode='mpc',record=False,progress=False):
    robot=Duck();m,d=robot.model,robot.data
    robot.initialize(ref.com[0],ref.feet[0]);start=d.qpos[:3].copy()
    offset=np.zeros(2);vfilter=np.zeros(2);anchor=np.zeros(2);previous=ref.zmp[0].copy();phase=None
    logs=[];qrecord=[];failure=None;target=robot.qcommand.copy();qp_seconds=0.;qp_res=0.;ik_error=0.
    initial_q=d.qpos.copy();touchpos=d.site_xpos[robot.feet,:2].copy();airtime=np.zeros(2);contacttime=np.zeros(2);armed=np.zeros(2,dtype=bool);peakheight=np.zeros(2);lastland=-1;landings=[]
    solve_times=[]
    initial_height=d.qpos[2];maxforce=np.max(np.abs(m.actuator_forcerange[:,1]));start_clock=time.perf_counter()
    for tick in range(int(round(ref.time[-1]/.002))):
        t=tick*.002;i=ref.index(t)
        if tick%5==0:
            com,vel=robot.state();vfilter=.5*vel[:2]+.5*vfilter
            now=tuple(ref.contacts[i])
            if now!=phase:
                stance=np.flatnonzero(ref.contacts[i]);anchor=np.mean(d.site_xpos[robot.feet[stance],:2]-ref.feet[i,stance,:2],axis=0);phase=now
            if tick%10==0 and controller is not None:
                states,zmp_ref,polygons=ref.preview(t,anchor=anchor)
                try:
                    p,pred,res=controller(np.r_[com[:2],vfilter],states,zmp_ref,polygons,previous,ref.height,.02)
                except (RuntimeError,ValueError) as exc:failure=f'QP: {exc}';break
                previous=p;qp_seconds=res.seconds;qp_res=res.residual;solve_times.append(res.seconds)
                # Bounded kinematic compliance: ZMP correction is not a motor torque.
                # Moving the CoM target opposite the ZMP correction provides a small
                # dynamic correction around the nominal gait. This is an inner-loop
                # approximation, not exact ZMP tracking or whole-body optimality.
                desired=np.clip(-.15*(p-zmp_ref[0]),-.004,.004)
                offset=.8*offset+.2*desired
            ctarget=ref.com[i].copy();ctarget[:2]+=anchor+offset
            ftarget=ref.feet[i].copy();ftarget[:,:2]+=anchor
            # Measured-state seed; the virtual IK base is never copied into simulation.
            q,ik_error=robot.ik(ctarget,ftarget,iterations=12)
            if not np.isfinite(q).all() or ik_error>.01:failure=f'IK error {ik_error:g}';break
            target=q[robot.qadr]
        robot.step(target)
        contacts,bad=robot.contacts();com,vel=robot.state()
        tilt=float(np.arccos(np.clip(d.xmat[1,8],-1,1)))
        for foot in range(2):
            if not contacts[foot]:
                airtime[foot]+=.002;contacttime[foot]=0
                peakheight[foot]=max(peakheight[foot],d.site_xpos[robot.feet[foot],2])
                if airtime[foot]>=.03 and peakheight[foot]>=.007:armed[foot]=True
            else:
                contacttime[foot]+=.002
                if armed[foot] and contacttime[foot]>=.02:
                    pos=d.site_xpos[robot.feet[foot],:2].copy();travel=pos[0]-touchpos[foot,0]
                    if travel>=.004 and foot!=lastland:
                        landings.append(dict(time=float(d.time),foot=int(foot),travel_m=float(travel)));lastland=foot
                    touchpos[foot]=pos;armed[foot]=False;peakheight[foot]=0
                airtime[foot]=0
        if not np.isfinite(d.qpos).all() or bad or tilt>np.deg2rad(30) or d.qpos[2]<.65*initial_height:
            failure=f'Physical failure at {d.time:.3f}s (nonfoot contact={bad}, tilt={np.rad2deg(tilt):.2f})';break
        if tick%5==0:
            saturation=np.mean(np.abs(d.actuator_force)>=.98*m.actuator_forcerange[:,1])
            qerr=np.sqrt(np.mean((d.qpos[robot.qadr]-target)**2))
            joint_violation=np.max(np.maximum(np.maximum(robot.low-.001-d.qpos[robot.qadr],d.qpos[robot.qadr]-robot.high-.001),0))
            logs.append(np.r_[d.time,d.qpos[:3],com,ref.com[i]+np.r_[anchor,0],tilt,contacts,ref.contacts[i],qerr,saturation,qp_seconds,qp_res,ik_error,offset,joint_violation])
            if record:qrecord.append(d.qpos.copy())
        if progress and tick%500==0:print(mode,f'{t:.1f}s',f'distance={d.qpos[0]-start[0]:.3f}m',f'landings={len(landings)}',flush=True)
    logs=np.asarray(logs)
    after_start=[x for x in landings if x['time']>=1.]
    distance=float(d.qpos[0]-start[0]);elapsed=float(d.time)
    startup_index=min(int(np.searchsorted(logs[:,0],1.)),len(logs)-1)
    scored_distance=float(d.qpos[0]-logs[startup_index,1])
    summary=dict(mode=mode,success=bool(failure is None and elapsed>=7 and len(after_start)>=8 and scored_distance>=.08),failure=failure,duration_s=elapsed,
                 forward_distance_m=distance,forward_distance_after_start_m=scored_distance,alternating_landings_after_start=len(after_start),landings=landings,
                 com_rmse_m=float(np.sqrt(np.mean((logs[:,4:7]-logs[:,7:10])**2))),joint_rmse_rad=float(np.sqrt(np.mean(logs[:,15]**2))),
                 max_tilt_deg=float(np.rad2deg(logs[:,10].max())),min_base_height_ratio=float(logs[:,3].min()/initial_height),
                 contact_disagreement=float(np.mean(logs[:,11:13]!=logs[:,13:15])),saturation_fraction=float(logs[:,16].mean()),
                 mean_qp_ms=float(np.mean(solve_times)*1000) if solve_times else 0.,max_qp_residual=float(logs[:,18].max()),max_ik_error_m=float(logs[:,19].max()),
                 max_joint_violation_rad=float(logs[:,-1].max()),wall_seconds=time.perf_counter()-start_clock)
    return dict(summary=summary,log=logs,qpos=np.asarray(qrecord),initial_qpos=initial_q)


def save_result(result,directory):
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True);mode=result['summary']['mode']
    (directory/f'{mode}_metrics.json').write_text(json.dumps(result['summary'],indent=2,allow_nan=False)+'\n')
    np.savez_compressed(directory/f'{mode}_log.npz',log=result['log'],qpos=result['qpos'],initial_qpos=result['initial_qpos'])


def plot_results(results):
    fig,axs=plt.subplots(3,2,figsize=(12,10))
    for result in results:
        a=result['log'];name=result['summary']['mode'];t=a[:,0]
        axs[0,0].plot(t,a[:,1]-a[0,1],label=name)
        axs[0,1].plot(t,np.linalg.norm(a[:,4:7]-a[:,7:10],axis=1),label=name)
        axs[1,0].plot(t,np.rad2deg(a[:,10]),label=name);axs[1,1].plot(t,a[:,15],label=name)
        axs[2,0].plot(t,a[:,17]*1000,label=name);axs[2,1].plot(t,a[:,16],label=name)
    for ax,title in zip(axs.ravel(),['Forward distance (m)','CoM tracking error (m)','Trunk tilt (deg)','Joint RMS error (rad)','QP solve time (ms)','Saturated actuator fraction']):
        ax.set(title=title,xlabel='Time (s)');ax.grid(alpha=.3);ax.legend()
    fig.tight_layout();return fig


def render_video(result,path):
    """Render recorded physical states after the experiment; never used for control."""
    import imageio.v2 as imageio
    robot=Duck();m,d=robot.model,robot.data
    camera=mujoco.MjvCamera();camera.distance=.9;camera.azimuth=140;camera.elevation=-15
    with mujoco.Renderer(m,height=480,width=640) as renderer,imageio.get_writer(str(path),fps=25) as writer:
        for q in result['qpos'][::4]:
            d.qpos[:]=q;mujoco.mj_forward(m,d);camera.lookat[:]=d.xpos[1];camera.lookat[2]+=.07
            renderer.update_scene(d,camera);writer.append_data(renderer.render())
