"""Compare six CasADi integration methods for a double pendulum or cart-pole.

Run cart-pole: python lecture2/q1_integration.py --model cartpole --animate
Verify headlessly: python lecture2/q1_integration.py --model cartpole --check --no-show
Dependencies: casadi, numpy, matplotlib.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np


@dataclass(frozen=True)
class Parameters:
    m1: float = 1.0
    m2: float = 1.0
    L1: float = 1.0
    L2: float = 1.0
    g: float = 9.8


PARAMS = Parameters()
X0 = np.array([np.pi / 1.6, 0.0, np.pi / 1.8, 0.0])


def double_pendulum_dynamics(params, x):
    """Continuous equations of motion."""
    theta1, omega1, theta2, omega2 = (x[i] for i in range(4))
    m1, m2, L1, L2, g = params.m1, params.m2, params.L1, params.L2, params.g
    c, s = ca.cos(theta1 - theta2), ca.sin(theta1 - theta2)
    alpha1 = (m2 * g * ca.sin(theta2) * c
              - m2 * s * (L1 * c * omega1**2 + L2 * omega2**2)
              - (m1 + m2) * g * ca.sin(theta1)) / (L1 * (m1 + m2 * s**2))
    alpha2 = ((m1 + m2) * (L1 * omega1**2 * s - g * ca.sin(theta2)
                           + g * ca.sin(theta1) * c)
              + m2 * L2 * omega2**2 * s * c) / (L2 * (m1 + m2 * s**2))
    return ca.vertcat(omega1, alpha1, omega2, alpha2)


def double_pendulum_energy(params, x):
    """Kinetic + potential energy, retaining 2 m pivot height."""
    theta1, omega1, theta2, omega2 = (x[i] for i in range(4))
    m1, m2, L1, L2, g = params.m1, params.m2, params.L1, params.L2, params.g
    kinetic = (0.5 * (m1 + m2) * L1**2 * omega1**2
               + 0.5 * m2 * L2**2 * omega2**2
               + m2 * L1 * L2 * omega1 * omega2 * ca.cos(theta1 - theta2))
    potential = ((m1 + m2) * g * (2 - L1 * ca.cos(theta1))
                 - m2 * g * L2 * ca.cos(theta2))
    return kinetic + potential


@dataclass(frozen=True)
class CartPoleParameters:
    M: float = 1.0  # cart mass (kg)
    m: float = 0.1  # point mass at the pole tip (kg)
    L: float = 1.0  # massless pole length (m)
    g: float = 9.8


CARTPOLE_PARAMS = CartPoleParameters()
CARTPOLE_X0 = np.array([0.0, 0.0, 0.1, 0.0])


def cartpole_dynamics(params, x, force=0.0):
    """State [position, velocity, theta, omega]; theta=0 is upright.

    Positive theta tilts the pole toward positive cart position. Force acts
    horizontally on the cart (N). The integration demo uses zero force,
    without friction or a balancing controller.
    """
    velocity, theta, omega = x[1], x[2], x[3]
    M, m, L, g = params.M, params.m, params.L, params.g
    s, c = ca.sin(theta), ca.cos(theta)
    acceleration = (force + m * s * (L * omega**2 - g * c)) / (M + m * s**2)
    alpha = (g * s - c * acceleration) / L
    return ca.vertcat(velocity, acceleration, omega, alpha)


def cartpole_energy(params, x):
    """Mechanical energy with potential zero at cart height."""
    velocity, theta, omega = x[1], x[2], x[3]
    M, m, L, g = params.M, params.m, params.L, params.g
    return (0.5 * (M + m) * velocity**2 + 0.5 * m * L**2 * omega**2
            + m * L * velocity * omega * ca.cos(theta) + m * g * L * ca.cos(theta))


# Part A: explicit update equations.
def forward_euler(params, dynamics, x, dt):
    return x + dt * dynamics(params, x)


def midpoint(params, dynamics, x, dt):
    xm = x + 0.5 * dt * dynamics(params, x)
    return x + dt * dynamics(params, xm)


def rk4(params, dynamics, x, dt):
    k1 = dt * dynamics(params, x)
    k2 = dt * dynamics(params, x + k1 / 2)
    k3 = dt * dynamics(params, x + k2 / 2)
    k4 = dt * dynamics(params, x + k3)
    return x + (k1 + 2 * k2 + 2 * k3 + k4) / 6


# Part B: implicit equations return a residual to be driven to zero.
def backward_euler(params, dynamics, x1, x2, dt):
    return x1 + dt * dynamics(params, x2) - x2


def implicit_midpoint(params, dynamics, x1, x2, dt):
    return x1 + dt * dynamics(params, (x1 + x2) / 2) - x2


def hermite_simpson(params, dynamics, x1, x2, dt):
    f1, f2 = dynamics(params, x1), dynamics(params, x2)
    xm = (x1 + x2) / 2 + dt * (f1 - f2) / 8
    return x1 + dt * (f1 + 4 * dynamics(params, xm) + f2) / 6 - x2


EXPLICIT = (forward_euler, midpoint, rk4)
IMPLICIT = (backward_euler, implicit_midpoint, hermite_simpson)
METHODS = EXPLICIT + IMPLICIT


def build_implicit_functions(params, dynamics, integrator, size):
    x1, x2, dt = ca.SX.sym('x1', size), ca.SX.sym('x2', size), ca.SX.sym('dt')
    residual = integrator(params, dynamics, x1, x2, dt)
    return ca.Function(integrator.__name__ + '_residual', [x1, x2, dt],
                       [residual, ca.jacobian(residual, x2)])


def implicit_integrator_solve(params, dynamics, integrator, x1, dt,
                              tol=1e-13, max_iters=10, functions=None):
    """Newton iterations using an exact CasADi automatic-differentiation Jacobian."""
    if functions is None:
        functions = build_implicit_functions(params, dynamics, integrator, len(x1))
    x2 = np.array(x1, dtype=float, copy=True)
    # Check the residual after the last permitted Newton update as well.
    for iteration in range(max_iters + 1):
        residual, jacobian = functions(x1, x2, dt)
        residual = np.asarray(residual).ravel()
        if np.all(np.isfinite(residual)) and np.linalg.norm(residual) < tol:
            return x2
        if iteration == max_iters or not np.all(np.isfinite(residual)):
            break
        x2 -= np.linalg.solve(np.asarray(jacobian), residual)
    raise RuntimeError(f'{integrator.__name__}: Newton solve failed; '
                       f'residual norm = {np.linalg.norm(residual):.3e}')


def time_grid(dt, tf):
    if not np.isfinite(dt) or not np.isfinite(tf) or dt <= 0 or tf < 0:
        raise ValueError('dt must be positive and tf nonnegative, both finite')
    # Like Julia's 0:dt:tf, omit any fractional final step.
    return np.arange(int(np.floor(tf / dt + 1e-10)) + 1) * dt


def _simulate(params, dynamics, integrator, x0, dt, tf, implicit, tol, energy_fn):
    times = time_grid(dt, tf)
    X = np.zeros((len(times), len(x0)))
    X[0] = x0
    x, h = ca.SX.sym('x', len(x0)), ca.SX.sym('dt')
    energy = ca.Function('energy', [x], [energy_fn(params, x)])
    # Build symbolic graphs once, outside the simulation loop.
    if implicit:
        functions = build_implicit_functions(params, dynamics, integrator, len(x0))
    else:
        step = ca.Function(integrator.__name__, [x, h], [integrator(params, dynamics, x, h)])
    for k in range(len(times) - 1):
        if implicit:
            X[k + 1] = implicit_integrator_solve(
                params, dynamics, integrator, X[k], dt, tol=tol, functions=functions)
        else:
            X[k + 1] = np.asarray(step(X[k], dt)).ravel()
        if not np.all(np.isfinite(X[k + 1])):
            raise RuntimeError(f'{integrator.__name__}: nonfinite state at step {k + 1}')
    E = np.array([float(energy(state)) for state in X])
    return X, E


def simulate_explicit(params, dynamics, integrator, x0, dt, tf,
                      energy_fn=double_pendulum_energy):
    return _simulate(params, dynamics, integrator, x0, dt, tf, False, 1e-13, energy_fn)


def simulate_implicit(params, dynamics, integrator, x0, dt, tf, tol=1e-13,
                      energy_fn=double_pendulum_energy):
    return _simulate(params, dynamics, integrator, x0, dt, tf, True, tol, energy_fn)


def simulate(integrator, dt=0.01, tf=2.0, model='double-pendulum'):
    simulator = simulate_implicit if integrator in IMPLICIT else simulate_explicit
    if model == 'cartpole':
        return simulator(CARTPOLE_PARAMS, cartpole_dynamics, integrator, CARTPOLE_X0,
                         dt, tf, energy_fn=cartpole_energy)
    if model != 'double-pendulum':
        raise ValueError(f'Unknown model: {model}')
    return simulator(PARAMS, double_pendulum_dynamics, integrator, X0, dt, tf)


def max_err_E(E):
    return float(np.max(np.abs(E - E[0])))


def run_checks(results, demo):
    """Reproduce the notebook's residual and energy-behavior checks."""
    for integrator in IMPLICIT:
        x1 = np.array([0.1, 0.2, 0.3, 0.4])
        x2 = implicit_integrator_solve(PARAMS, double_pendulum_dynamics, integrator, x1, 0.1)
        residual = integrator(PARAMS, double_pendulum_dynamics, ca.DM(x1), ca.DM(x2), 0.1)
        assert np.linalg.norm(np.asarray(residual)) < 1e-10
    X, E = demo
    assert np.linalg.norm(X[-1]) > 1e-10
    assert 2 < E[-1] / E[0] < 3
    changes = {method: E[-1] - E[0] for method, (_, E) in results.items()}
    assert 2.5 < changes[forward_euler] < 3.0
    assert -3.0 < changes[backward_euler] < -2.5
    for method, bound in [(implicit_midpoint, 1e-2), (hermite_simpson, 1e-4),
                          (midpoint, 1e-1), (rk4, 1e-4)]:
        assert abs(changes[method]) < bound, method.__name__
    print('All notebook checks passed.')


def run_cartpole_checks(results):
    params = CARTPOLE_PARAMS
    np.testing.assert_allclose(cartpole_dynamics(params, np.zeros(4)), 0, atol=1e-14)
    # An upright perturbation accelerates away from equilibrium.
    assert float(cartpole_dynamics(params, CARTPOLE_X0)[3]) > 0
    # Check the coupled equations and power balance for a forced state.
    x, u = ca.SX.sym('x', 4), ca.SX.sym('u')
    f = cartpole_dynamics(params, x, u)
    energy_rate = ca.Function('energy_rate', [x, u],
                              [ca.jacobian(cartpole_energy(params, x), x) @ f])
    state, force = np.array([0.2, 0.3, 0.4, -0.5]), 1.7
    rate = float(energy_rate(state, force))
    np.testing.assert_allclose(rate, force * state[1], atol=1e-12)
    acceleration = np.asarray(cartpole_dynamics(params, state, force)).ravel()
    theta, omega = state[2:]
    mass_matrix = np.array([[params.M + params.m, params.m * params.L * np.cos(theta)],
                            [params.m * params.L * np.cos(theta), params.m * params.L**2]])
    rhs = [force + params.m * params.L * omega**2 * np.sin(theta),
           params.m * params.g * params.L * np.sin(theta)]
    np.testing.assert_allclose(mass_matrix @ acceleration[[1, 3]], rhs, atol=1e-12)
    for integrator in IMPLICIT:
        x2 = implicit_integrator_solve(params, cartpole_dynamics, integrator, state, 0.1)
        residual = integrator(params, cartpole_dynamics, ca.DM(state), ca.DM(x2), 0.1)
        assert np.linalg.norm(np.asarray(residual)) < 1e-10
    for X, E in results.values():
        assert np.all(np.isfinite(X)) and np.all(np.isfinite(E))
        np.testing.assert_array_equal(X[0], CARTPOLE_X0)
    assert max_err_E(results[rk4][1]) < 1e-5
    print('All cart-pole checks passed.')


def animate_cartpole(X, dt):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Rectangle

    L = CARTPOLE_PARAMS.L
    fig, ax = plt.subplots()
    ax.set(xlim=(X[:, 0].min() - L - 0.5, X[:, 0].max() + L + 0.5),
           ylim=(-L - 0.3, L + 0.3), xlabel='Position (m)', ylabel='Height (m)',
           title='Forward Euler: unforced cart-pole')
    ax.set_aspect('equal')
    ax.axhline(0, color='gray', lw=1)
    cart = Rectangle((0, -0.1), 0.4, 0.2, color='tab:blue')
    ax.add_patch(cart)
    pole, = ax.plot([], [], 'o-', lw=2, color='tab:orange')
    clock = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def update(k):
        position, theta = X[k, 0], X[k, 2]
        cart.set_x(position - 0.2)
        pole.set_data([position, position + L * np.sin(theta)], [0, L * np.cos(theta)])
        clock.set_text(f't = {k * dt:.2f} s')
        return cart, pole, clock

    stride = max(1, round(1 / (30 * dt)))
    return FuncAnimation(fig, update, frames=range(0, len(X), stride),
                         interval=1000 * dt * stride, blit=True)


def animate_pendulum(X, dt):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    fig, ax = plt.subplots()
    reach = PARAMS.L1 + PARAMS.L2
    ax.set(xlim=(-reach - 0.1, reach + 0.1), ylim=(2 - reach - 0.1, 2 + reach + 0.1),
           xlabel='x (m)', ylabel='z (m)', title='Forward Euler: double pendulum')
    ax.set_aspect('equal')
    line, = ax.plot([], [], 'o-', lw=2)
    clock = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def update(k):
        a, b = X[k, 0], X[k, 2]
        x1, z1 = PARAMS.L1 * np.sin(a), 2 - PARAMS.L1 * np.cos(a)
        line.set_data([0, x1, x1 + PARAMS.L2 * np.sin(b)],
                      [2, z1, z1 - PARAMS.L2 * np.cos(b)])
        clock.set_text(f't = {k * dt:.2f} s')
        return line, clock

    stride = max(1, round(1 / (30 * dt)))
    return FuncAnimation(fig, update, frames=range(0, len(X), stride),
                         interval=1000 * dt * stride, blit=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', choices=['double-pendulum', 'cartpole'],
                        default='double-pendulum', help='dynamics to simulate')
    parser.add_argument('--check', action='store_true', help='run checks for the selected model')
    parser.add_argument('--no-show', action='store_true', help='run without opening windows')
    parser.add_argument('--animate', action='store_true', help='animate the 30 s Euler experiment')
    parser.add_argument('--output-dir', type=Path, help='save plots as PNG files')
    args = parser.parse_args()
    if args.animate and args.no_show:
        parser.error('--animate requires interactive display; omit --no-show')
    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    demo = simulate(forward_euler, tf=30.0, model=args.model)
    results = {method: simulate(method, model=args.model) for method in METHODS}
    if args.check:
        if args.model == 'cartpole':
            run_cartpole_checks(results)
        else:
            run_checks(results, demo)

    figures = {}
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(9, 7))
    axes[0].plot(time_grid(0.01, 30), demo[0])
    labels = (['position (m)', 'velocity (m/s)', 'theta (rad)', 'omega (rad/s)']
              if args.model == 'cartpole' else ['theta1', 'omega1', 'theta2', 'omega2'])
    axes[0].legend(labels)
    axes[0].set(ylabel='State', title=f'{args.model}: Forward Euler, dt = 0.01 s')
    axes[1].plot(time_grid(0.01, 30), demo[1])
    axes[1].set(xlabel='Time (s)', ylabel='Energy (J)')
    figures['forward_euler'] = fig

    fig, ax = plt.subplots(figsize=(9, 5))
    print(f'{"Integrator":<22} {"Final energy change (J)":>24} {"Max energy error (J)":>22}')
    for method, (_, E) in results.items():
        label = method.__name__.replace('_', ' ').title()
        ax.plot(time_grid(0.01, 2), E, label=label)
        print(f'{label:<22} {E[-1] - E[0]:24.8g} {max_err_E(E):22.8g}')
    ax.set(xlabel='Time (s)', ylabel='Energy (J)', title='Energy behavior, dt = 0.01 s')
    ax.legend()
    figures['energy_behavior'] = fig

    fig, ax = plt.subplots(figsize=(9, 5))
    dts = [1e-3, 1e-2, 1e-1]
    for method in METHODS:
        errors = [max_err_E(simulate(method, dt=dt, model=args.model)[1]) for dt in dts]
        ax.loglog(dts, errors, 'o--' if method in IMPLICIT else 'o-',
                  label=method.__name__.replace('_', ' ').title())
    ax.set(xlabel='Time step (s)', ylabel='Maximum energy error (J)',
           title='Integrator comparison over 2 seconds')
    ax.legend()
    ax.grid(True, which='both', alpha=0.3)
    figures['energy_error'] = fig
    for name, fig in figures.items():
        fig.tight_layout()
        if args.output_dir:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            fig.savefig(args.output_dir / f'{args.model}_{name}.png', dpi=150)

    # Part C: Forward Euler adds energy and becomes unstable; backward Euler
    # dissipates energy, while the midpoint methods, RK4, and Hermite-Simpson
    # have much smaller energy errors in this experiment.
    animate = animate_cartpole if args.model == 'cartpole' else animate_pendulum
    animation = animate(demo[0], 0.01) if args.animate else None
    if not args.no_show:
        plt.show()  # Keep the animation referenced until its window closes.
    plt.close('all')


if __name__ == '__main__':
    main()
