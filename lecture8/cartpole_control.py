"""Given model-based tools for HW7 Part D: LQR/MPC on the *same* Gymnasium CartPole-v1.

* ``cartpole_step`` reproduces one ``CartPoleEnv.step`` (Euler, 0.02 s) for a continuous force,
  so it can be linearized exactly like the Lecture 4 model.
* ``ContinuousForceCartPole`` lets a controller apply any force in [-10, 10] N to the real
  environment by setting ``force_mag`` for one step. Integration, termination and the
  500-step limit stay those of CartPole-v1; a discrete push of ±10 N is the special case |F| = 10.
* ``lqr_force_policy`` is the clipped discrete LQR of Lectures 4 and 7 (DARE + saturation).
* ``CondensedMPC`` is an input-constrained linear MPC solved with OSQP. It uses the condensed
  form (states eliminated), unlike the sparse full-state QP of Lecture 7.
"""
from dataclasses import dataclass
from time import perf_counter
import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
from scipy import sparse
from scipy.linalg import solve_discrete_are
import osqp

MAX_FORCE = 10.0


@dataclass(frozen=True)
class CartPoleParams:
    gravity: float = 9.8
    masscart: float = 1.0
    masspole: float = 0.1
    length: float = 0.5   # half the pole length, as in Gymnasium
    tau: float = 0.02

    @classmethod
    def from_env(cls, env):
        u = env.unwrapped
        return cls(u.gravity, u.masscart, u.masspole, u.length, u.tau)


def cartpole_step(state, force, params=CartPoleParams()):
    """One Euler step of CartPole-v1 with a continuous horizontal force (N)."""
    x, x_dot, theta, theta_dot = np.asarray(state, dtype=float)
    force = float(np.asarray(force).reshape(-1)[0])
    p = params
    total_mass = p.masspole + p.masscart
    polemass_length = p.masspole * p.length
    c, s = np.cos(theta), np.sin(theta)
    temp = (force + polemass_length * theta_dot**2 * s) / total_mass
    thetaacc = (p.gravity * s - c * temp) / (p.length * (4.0 / 3.0 - p.masspole * c**2 / total_mass))
    xacc = temp - polemass_length * thetaacc * c / total_mass
    return np.array([x + p.tau * x_dot, x_dot + p.tau * xacc, theta + p.tau * theta_dot, theta_dot + p.tau * thetaacc])


class ContinuousForceCartPole(gym.Wrapper):
    """CartPole-v1 driven by a force in newtons, clipped to [-max_force, max_force].

    Wrap this *outside* ``PerturbedCartPole``: the force set here replaces ``force_mag``.
    The applied force is reported as ``info['force']``.
    """

    def __init__(self, env, max_force=MAX_FORCE):
        super().__init__(env)
        self.max_force = float(max_force)
        self.action_space = gym.spaces.Box(np.float32(-self.max_force), np.float32(self.max_force), (1,), np.float32)

    def step(self, action):
        force = float(np.clip(np.asarray(action, dtype=float).reshape(-1)[0], -self.max_force, self.max_force))
        u = self.unwrapped
        nominal = u.force_mag
        u.force_mag = abs(force)
        try:
            obs, reward, terminated, truncated, info = self.env.step(1 if force >= 0 else 0)
        finally:
            u.force_mag = nominal
        return obs, reward, terminated, truncated, {**info, 'force': force}


def continuous_training_env(env):
    """Wrapper chain for PPO with a continuous action a in [-1, 1] meaning F = 10 a newtons."""
    return gym.wrappers.RescaleAction(ContinuousForceCartPole(env), np.float32(-1.0), np.float32(1.0))


class InitialStateWrapper(gym.Wrapper):
    """Start every episode from a fixed state instead of Gymnasium's U(-0.05, 0.05)."""

    def __init__(self, env, state):
        super().__init__(env)
        self.initial_state = np.asarray(state, dtype=np.float64).reshape(4)

    def reset(self, **kwargs):
        _, info = self.env.reset(**kwargs)
        self.unwrapped.state = self.initial_state.copy()
        return np.array(self.initial_state, dtype=np.float32), info


def discrete_as_force(policy, max_force=MAX_FORCE):
    """Turn a 0/1 CartPole policy into a ±max_force policy for ContinuousForceCartPole."""
    return lambda obs: max_force if int(policy(obs)) == 1 else -max_force


def lqr_force_policy(A, B, Q, R, max_force=MAX_FORCE):
    """Clipped discrete LQR (Lectures 4 and 7). Returns (policy, K, P).

    P solves the DARE, K = (R + B^T P B)^{-1} B^T P A, and policy(obs) = float(clip(-K obs)).
    """
    P = solve_discrete_are(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    policy = lambda obs: float(np.clip(-(K @ np.asarray(obs, dtype=float))[0], -max_force, max_force))
    return policy, K, P


class Timed:
    """Callable wrapper that records the wall-clock time of every policy call."""

    def __init__(self, policy):
        self.policy, self.seconds = policy, []

    def __call__(self, obs):
        t = perf_counter()
        action = self.policy(obs)
        self.seconds.append(perf_counter() - t)
        return action

    @property
    def mean_ms(self):
        return 1e3 * float(np.mean(self.seconds)) if self.seconds else float('nan')


class CondensedMPC:
    """Linear MPC with a hard force limit and soft state limits, solved by OSQP at every call.

    Prediction x_{i+1} = A x_i + B u_i is condensed to X = Phi x0 + Gamma U, with
    X = [x_1..x_N], and the cost sum_{i<N}(x_i'Q x_i + u_i'R u_i) + x_N'P x_N becomes
    0.5 U'HU + (F x0)'U + const. ``state_limits`` (4 numbers, ``np.inf`` = unconstrained)
    bounds the predicted states through slacks e >= 0 penalized by ``slack_weight * (e + e^2)``,
    so the QP stays feasible even when the measured state is already outside the limits.
    A solver failure raises ``RuntimeError``; it is never replaced by a guessed action.
    """

    def __init__(self, A, B, Q, R, P, N, max_force=MAX_FORCE, state_limits=None, slack_weight=1e3):
        A, B = np.asarray(A, float), np.asarray(B, float).reshape(4, -1)
        n, m = B.shape
        if N < 1:
            raise ValueError('N must be positive')
        Phi = np.vstack([np.linalg.matrix_power(A, i + 1) for i in range(N)])
        Gamma = np.zeros((N * n, N * m))
        for i in range(N):
            for j in range(i + 1):
                Gamma[i * n:(i + 1) * n, j * m:(j + 1) * m] = np.linalg.matrix_power(A, i - j) @ B
        Qbar = np.kron(np.eye(N), Q)
        Qbar[-n:, -n:] = P
        Rbar = np.kron(np.eye(N), R)
        H = 2 * (Gamma.T @ Qbar @ Gamma + Rbar)
        limits = np.full(n, np.inf) if state_limits is None else np.asarray(state_limits, float)
        rows = [i * n + k for i in range(N) for k in range(n) if np.isfinite(limits[k])]
        ns, nu = len(rows), N * m
        self.N, self.m, self.nu, self.max_force = N, m, nu, float(max_force)
        self.F = np.vstack([2 * Gamma.T @ Qbar @ Phi, np.zeros((ns, n))])
        self.S_phi, self.bound = Phi[rows], np.tile(limits, N)[rows]
        self.q_slack = np.r_[np.zeros(nu), np.full(ns, slack_weight)]
        Hs = sparse.block_diag([sparse.csc_matrix(H), 2 * slack_weight * sparse.eye(ns)], format='csc')
        G = Gamma[rows]
        # rows: force box | G U - e <= bound - S x0 | G U + e >= -bound - S x0 | e >= 0
        self.G = sparse.vstack([sparse.hstack([sparse.eye(nu), sparse.csc_matrix((nu, ns))]),
                                sparse.hstack([sparse.csc_matrix(G), -sparse.eye(ns)]),
                                sparse.hstack([sparse.csc_matrix(G), sparse.eye(ns)]),
                                sparse.hstack([sparse.csc_matrix((ns, nu)), sparse.eye(ns)])], format='csc')
        self.ns = ns
        self.solver = osqp.OSQP()
        l, u = self._bounds(np.zeros(n))
        self.solver.setup(P=sparse.triu(Hs, format='csc'), q=self.q_slack.copy(), A=self.G, l=l, u=u,
                          verbose=False, eps_abs=1e-4, eps_rel=1e-4, polishing=False, warm_starting=True,
                          max_iter=4000)

    def _bounds(self, x0):
        free = self.bound - self.S_phi @ x0
        lo = np.r_[np.full(self.nu, -self.max_force), np.full(self.ns, -np.inf), -self.bound - self.S_phi @ x0, np.zeros(self.ns)]
        hi = np.r_[np.full(self.nu, self.max_force), free, np.full(self.ns, np.inf), np.full(self.ns, np.inf)]
        return lo, hi

    def plan(self, x0):
        x0 = np.asarray(x0, dtype=float)
        lo, hi = self._bounds(x0)
        self.solver.update(q=self.q_slack + self.F @ x0, l=lo, u=hi)
        result = self.solver.solve(raise_error=False)
        if result.info.status_val not in (1, 2) or result.x is None or not np.isfinite(result.x).all():
            raise RuntimeError(f'MPC QP failed: {result.info.status}')
        self.last_slack = float(np.max(result.x[self.nu:], initial=0.0))
        return result.x[:self.nu].reshape(self.N, self.m)

    def __call__(self, obs):
        return float(self.plan(obs)[0, 0])


def plot_controller_traces(episodes, max_force=MAX_FORCE, dt=0.02, title=None, t_max=None):
    """``episodes``: {label: run_episode result on ContinuousForceCartPole}.

    Pure bang-bang force traces (|F| = max_force at every step) are drawn faintly so they do
    not hide the continuous controllers; ``t_max`` limits the time axis.
    """
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    for label, ep in episodes.items():
        X, F = np.asarray(ep['observations']), np.asarray(ep['actions'], dtype=float)
        t = np.arange(len(X)) * dt
        axs[0].plot(t, np.rad2deg(X[:, 2]), label=label)
        axs[1].plot(t, X[:, 0], label=label)
        bang_bang = len(F) > 0 and np.allclose(np.abs(F), max_force)
        axs[2].step(t[:-1], F, where='post', label=label, alpha=.25 if bang_bang else .9,
                    lw=.6 if bang_bang else 1.5, zorder=1 if bang_bang else 3)
    for s in (-1, 1):
        axs[0].axhline(s * 12, color='r', ls=':', alpha=.5)
        axs[1].axhline(s * 2.4, color='r', ls=':', alpha=.5)
        axs[2].axhline(s * max_force, color='k', ls=':', alpha=.5)
    for ax, y in zip(axs, ['Pole angle (deg)', 'Cart position (m)', 'Force (N)']):
        ax.set_ylabel(y)
        ax.grid(alpha=.3)
    axs[0].legend(ncol=3, fontsize=8)
    axs[-1].set_xlabel('Time (s)')
    if t_max is not None:
        axs[-1].set_xlim(0, t_max)
    if title:
        axs[0].set_title(title)
    fig.tight_layout()
    return fig
