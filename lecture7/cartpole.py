"""Given nonlinear wheel-driven cart-pole from Lecture 4 (SI units).

Parameters extracted at the 0.30 m pineapple equilibrium using the Lecture 4
PineappleModel. Input is total wheel torque, NOT horizontal cart force.
"""
from dataclasses import dataclass
import numpy as np
from scipy.linalg import expm, solve_discrete_are

@dataclass(frozen=True)
class Parameters:
    M: float=5.836646
    length: float=.20985764075258312
    inertia: float=.08745507558118053
    wheel_mass: float=.6
    wheel_inertia: float=.001868
    radius: float=.07
    gravity: float=9.81
    @property
    def mass_matrix(self):
        return np.array([[self.M+self.wheel_mass+self.wheel_inertia/self.radius**2,self.M*self.length],
                         [self.M*self.length,self.inertia+self.M*self.length**2]])

PARAMS=Parameters()

def dynamics(x,u,params=PARAMS):
    s,theta,v,w=np.asarray(x,dtype=float)
    u=float(np.asarray(u).item());M=params.mass_matrix.copy();b=params.M*params.length
    M[0,1]=M[1,0]=b*np.cos(theta)
    acc=np.linalg.solve(M,[b*w*w*np.sin(theta)+u/params.radius,b*params.gravity*np.sin(theta)-u])
    return np.r_[v,w,acc]

def rk4(x,u,dt,params=PARAMS):
    if dt<=0:raise ValueError('dt must be positive')
    x=np.asarray(x,dtype=float)
    a=dynamics(x,u,params);b=dynamics(x+dt*a/2,u,params);c=dynamics(x+dt*b/2,u,params);d=dynamics(x+dt*c,u,params)
    return x+dt*(a+2*b+2*c+d)/6

def linear_model(dt=.02,params=PARAMS):
    A=np.zeros((4,4));B=np.zeros((4,1));A[0,2]=A[1,3]=1
    A[2:,1]=np.linalg.solve(params.mass_matrix,[0,params.M*params.gravity*params.length])
    B[2:,0]=np.linalg.solve(params.mass_matrix,[1/params.radius,-1])
    if dt<=0:raise ValueError('dt must be positive')
    # Same physical model as HW3; exact ZOH instead of differentiating an RK4 map.
    aug=np.zeros((5,5));aug[:4,:4]=A;aug[:4,4:]=B
    E=expm(aug*dt)
    return A,B,E[:4,:4],E[:4,4:]

def lqr(Ad,Bd,Q,R):
    P=solve_discrete_are(Ad,Bd,Q,R)
    K=np.linalg.solve(R+Bd.T@P@Bd,Bd.T@P@Ad)
    return K,P
