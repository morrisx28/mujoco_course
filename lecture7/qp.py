"""Given OSQP interface; students supply the QP matrices."""
from dataclasses import dataclass
from time import perf_counter
import numpy as np
from scipy import sparse
import osqp

@dataclass
class QPResult:
    z: np.ndarray
    status: str
    seconds: float
    residual: float

def solve_qp(H,f,G,lower,upper,eps=1e-7,max_iter=20000,**settings):
    H=sparse.csc_matrix(H);G=sparse.csc_matrix(G)
    f=np.asarray(f,dtype=float);lower=np.asarray(lower,dtype=float);upper=np.asarray(upper,dtype=float)
    if H.shape!=(len(f),len(f)) or G.shape!=(len(lower),len(f)) or lower.shape!=upper.shape:
        raise ValueError('QP dimensions disagree')
    if np.any(lower>upper) or not np.isfinite(f).all() or np.isnan(lower).any() or np.isnan(upper).any():
        raise ValueError('Invalid QP bounds/data')
    solver=osqp.OSQP();t=perf_counter()
    solver.setup(P=sparse.triu(H,format='csc'),q=f,A=G,l=lower,u=upper,
                 verbose=False,eps_abs=eps,eps_rel=eps,max_iter=max_iter,
                 polishing=True,adaptive_rho_interval=50,**settings)
    result=solver.solve(raise_error=False)
    elapsed=perf_counter()-t
    if result.info.status_val!=1 or result.x is None or not np.isfinite(result.x).all():
        raise RuntimeError(f'QP failed: {result.info.status}')
    residual=float(max(0,np.max(lower-G@result.x),np.max(G@result.x-upper)))
    if residual>max(2e-5,20*eps):raise RuntimeError(f'QP constraint residual {residual:g}')
    return QPResult(result.x,result.info.status,elapsed,residual)
