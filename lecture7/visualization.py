"""Given plotting/animation utilities; all trajectories come from student functions."""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def plot_comparison(runs,ulimit):
    fig,axs=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for name,r in runs.items():
        t=np.arange(len(r['X']))*r['dt'];axs[0].plot(t,np.rad2deg(r['X'][:,1]),label=name)
        axs[1].plot(t,r['X'][:,0],label=name)
        axs[2].step(t[:-1],r['U'][:,0],where='post',label=name)
    for s in [-1,1]:axs[2].axhline(s*ulimit,color='k',ls=':',alpha=.6)
    for ax,y in zip(axs,['Pitch (deg)','Cart position (m)','Wheel torque (Nm)']):
        ax.set_ylabel(y);ax.grid(alpha=.3);ax.legend()
    axs[-1].set_xlabel('Time (s)');fig.tight_layout();return fig

def plot_windows(run,indices=(0,15,40),dt=.02):
    fig,axs=plt.subplots(1,len(indices),figsize=(14,3.8))
    for ax,k in zip(axs,indices):
        if k>=len(run['predictions']) or run['predictions'][k] is None:continue
        pred=run['predictions'][k];times=(k+np.arange(len(pred)))*dt
        ax.plot(times,np.rad2deg(pred[:,1]),'--',label='Prediction at k')
        end=min(k+len(pred),len(run['X']))
        ax.plot(np.arange(k,end)*dt,np.rad2deg(run['X'][k:end,1]),label='Executed / replanned')
        ax.axvspan(k*dt,(k+1)*dt,color='orange',alpha=.2,label='First input only')
        ax.set(title=f'Window at t={k*dt:.2f} s',xlabel='Time (s)',ylabel='Pitch (deg)');ax.grid(alpha=.3);ax.legend(fontsize=8)
    fig.tight_layout();return fig

def animate_cartpole(run):
    X=run['X'];fig,ax=plt.subplots(figsize=(7,3));ax.set(xlim=(X[:,0].min()-.4,X[:,0].max()+.4),ylim=(-.08,.35),xlabel='Position (m)',title='Wheel-driven cart-pole (kinematic drawing of simulated states)');ax.set_aspect('equal');ax.axhline(0,color='gray')
    cart,=ax.plot([],[],'ks',ms=12);rod,=ax.plot([],[],'o-',lw=3);label=ax.text(.02,.9,'',transform=ax.transAxes)
    def draw(k):
        s,theta=X[k,:2];cart.set_data([s],[0]);rod.set_data([s,s+.21*np.sin(theta)],[0,.21*np.cos(theta)]);label.set_text(f't={k*run["dt"]:.2f} s');return cart,rod,label
    animation=FuncAnimation(fig,draw,frames=range(0,len(X),2),interval=2000*run['dt'],blit=True);plt.close(fig);return animation

def metrics(run,Q,R,ulimit,xlimit):
    X,U=run['X'],run['U'];cost=np.einsum('bi,ij,bj->',X[:-1],Q,X[:-1])+np.einsum('bi,ij,bj->',U,R,U)
    return dict(cost=float(cost),peak_torque=float(np.max(np.abs(U),initial=0)),input_violation=float(np.maximum(np.abs(U)-ulimit,0).max(initial=0)),state_violation=float(np.maximum(np.abs(X)-xlimit,0).max(initial=0)),mean_solve_ms=float(np.mean(run['seconds'])*1000) if len(U) else 0.,failure=run['failure'])
