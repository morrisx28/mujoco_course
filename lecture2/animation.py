"""Given animation helpers for HW1. There are no TODOs in this file.

`animate_pendulum` and `animate_cartpole` build a matplotlib FuncAnimation from a
state trajectory. `show_animation` turns one into something a notebook can display.
"""

import numpy as np


def show_animation(animation, fps=20, max_frames=250):
    """Render a FuncAnimation for display in a notebook cell.

    Uses ffmpeg (small, fast) when it is installed and falls back to a pure
    JavaScript player otherwise, so no extra packages are required.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import writers
    from IPython.display import HTML

    if writers.is_available('ffmpeg'):
        html = HTML(animation.to_html5_video())
    else:
        html = HTML(animation.to_jshtml(fps=fps))
    plt.close(animation._fig)  # otherwise the notebook also shows a static frame
    return html


def _frames(n_states, dt, max_frames):
    """Subsample so the embedded animation stays small; aim for ~30 fps of wall time."""
    stride = max(1, round(1 / (30 * dt)), int(np.ceil(n_states / max_frames)))
    return range(0, n_states, stride), stride


def animate_pendulum(params, X, dt, max_frames=250, title='Double pendulum'):
    """Animate a double-pendulum trajectory; X[k] is [theta1, omega1, theta2, omega2]."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    X = np.asarray(X, dtype=float)
    fig, ax = plt.subplots()
    reach = params.L1 + params.L2
    ax.set(xlim=(-reach - 0.1, reach + 0.1), ylim=(2 - reach - 0.1, 2 + reach + 0.1),
           xlabel='x (m)', ylabel='z (m)', title=title)
    ax.set_aspect('equal')
    line, = ax.plot([], [], 'o-', lw=2)
    clock = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    def update(k):
        a, b = X[k, 0], X[k, 2]
        x1, z1 = params.L1 * np.sin(a), 2 - params.L1 * np.cos(a)
        line.set_data([0, x1, x1 + params.L2 * np.sin(b)],
                      [2, z1, z1 - params.L2 * np.cos(b)])
        clock.set_text(f't = {k * dt:.2f} s')
        return line, clock

    frames, stride = _frames(len(X), dt, max_frames)
    return FuncAnimation(fig, update, frames=frames,
                         interval=1000 * dt * stride, blit=True)


def animate_cartpole(params, X, dt, max_frames=250, title='Cart-pole'):
    """Animate a cart-pole trajectory; X[k] is [position, velocity, theta, omega]."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Rectangle

    X = np.asarray(X, dtype=float)
    L = params.L
    fig, ax = plt.subplots()
    ax.set(xlim=(X[:, 0].min() - L - 0.5, X[:, 0].max() + L + 0.5),
           ylim=(-L - 0.3, L + 0.3), xlabel='Position (m)', ylabel='Height (m)',
           title=title)
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

    frames, stride = _frames(len(X), dt, max_frames)
    return FuncAnimation(fig, update, frames=frames,
                         interval=1000 * dt * stride, blit=True)
