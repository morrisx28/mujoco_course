# HW2: Train Pineapple V0 to follow velocity commands with mjlab

**Deliverable:** one video (100 points). [中文版說明](README_zh_TW.md)

Pineapple V0 ([robot/pineapple_v0](../robot/pineapple_v0)) is a wheeled biped. Each leg has a thigh, a
calf and a wheel. With its legs held still and its wheels braked it tips over in under a second: it is
a wheeled inverted pendulum. In this homework you train a reinforcement-learning policy with
[mjlab](https://github.com/mujocolab/mjlab) (GPU MuJoCo Warp + PPO) that **keeps the robot balanced
and makes it follow a commanded forward speed and turning rate**. You then drive the trained policy
in play mode and film it. There is no sim-to-sim or hardware part.

The robot has no hip joint, so it cannot move sideways. A command is
$(v_x, \omega_z)$: forward speed in m/s and yaw rate in rad/s, both in the robot's frame.

## What is given and what you write

| File (copied into mjlab by the installer) | Status |
|---|---|
| `asset_zoo/robots/pineapple_v0/pineapple_v0_constants.py` | **Given.** Loads the course XML, adds a free joint, PD position servos for the legs (40 Nm/rad, 23.7 Nm limit), velocity servos for the wheels (3.69 Nm limit), the initial pose and joint-name constants. |
| `tasks/velocity/config/pineapple_v0/rl_cfg.py` | **Given.** PPO settings (you may tune them). |
| `tasks/velocity/config/pineapple_v0/env_cfgs.py` | **You write six functions**: `_actions`, `_commands`, `_observations`, `_rewards`, `_terminations`, `_events`. The scene, a ground-contact sensor for non-wheel bodies, simulation (200 Hz physics, 50 Hz policy) and play mode are given. |
| `tasks/velocity/config/pineapple_v0/__init__.py` | **You uncomment** the task registration once `env_cfgs.py` is done. |

Each TODO's docstring lists the mjlab functions to use. Read mjlab's generic velocity task
(`src/mjlab/tasks/velocity/velocity_env_cfg.py`), the Go1 example (`config/go1/env_cfgs.py`) and the term
libraries (`src/mjlab/tasks/velocity/mdp/`, `src/mjlab/envs/mdp/`). The design decisions are yours: which
observations, which reward terms and weights, which randomization. A wheeled robot differs from the
legged examples. It has no feet or gait, the wheels need velocity control, and wheel angles grow without bound.

## 1. Install (NVIDIA GPU required for training)

```bash
git clone https://github.com/mujocolab/mjlab.git
cd mjlab
git checkout v1.6.0                       # HW2 is tested on this version only
uv sync                                   # installs mjlab, MuJoCo Warp, PyTorch (CUDA)
python /path/to/mujoco_course/hw2/starter/install_into_mjlab.py --mjlab .
uv run python /path/to/mujoco_course/hw2/starter/check_setup.py
```

Install [uv](https://docs.astral.sh/uv/) first if needed. `check_setup.py` checks that the robot
compiles, that the legs hold their pose, and that the robot falls over with zero action (expected). It
also reports whether your task is registered. Re-running the installer never overwrites your task
files unless you pass `--overwrite-task`.

## 2. Write and check the task

1. Implement the six functions in `src/mjlab/tasks/velocity/config/pineapple_v0/env_cfgs.py`.
2. Uncomment the registration in `__init__.py` of that folder. Then `uv run list-envs` must show
   `Mjlab-Velocity-Flat-Pineapple-V0`, and `check_setup.py` prints your terms.
3. Sanity-check the MDP before training:
   ```bash
   uv run play Mjlab-Velocity-Flat-Pineapple-V0 --agent zero     # falls over: no balance yet
   uv run play Mjlab-Velocity-Flat-Pineapple-V0 --agent random
   ```

## 3. Train

```bash
uv run train Mjlab-Velocity-Flat-Pineapple-V0 --env.scene.num-envs 4096
uv run tensorboard --logdir logs/rsl_rl/pineapple_v0_velocity       # reward curves
```

Checkpoints are saved every 100 iterations in `logs/rsl_rl/pineapple_v0_velocity/<date_time>/model_<N>.pt`.
Use `--env.scene.num-envs 2048` if the GPU runs out of memory, and `--agent.max-iterations N` to change the
length of a run (3000 by default). Watch `Mean episode length` (balance; the maximum is 1000 steps = 20 s) and
the `track_*` reward terms (command following). The reference solution balanced within about 100
iterations. On an RTX A2000 laptop GPU it ran about 0.9 s per iteration with 4096 envs.
Train until `Mean episode length` stays at 1000 and the tracking rewards level off, then play the latest
checkpoint. If the robot does not follow the sliders well yet, keep training or tune your rewards.

If the robot balances but does not follow commands, look at your tracking rewards and command ranges.
If it never balances, look at terminations, the falling penalty and the action scales.

## 4. Play and film

```bash
uv run play Mjlab-Velocity-Flat-Pineapple-V0 --viewer viser \
    --checkpoint-file logs/rsl_rl/pineapple_v0_velocity/<date_time>/model_<N>.pt
```

Open the URL printed in the terminal (usually <http://localhost:8080>). On a remote GPU machine,
forward the port first: `ssh -L 8080:localhost:8080 user@server`. In the **Twist** panel tick **Enable**
and drive the robot with the `lin_vel_x` and `ang_vel_z` sliders. The arrows above the robot show the
command and the measured velocity. Record the browser window, including the slider panel, with any
screen recorder (OBS, Windows Game Bar `Win+Alt+R`, macOS `Cmd+Shift+5`).

### Video requirements

* One continuous take, no cuts or speed-up, **at least 60 s**, MP4, 720p or better.
* First 5 s: the terminal with your `play` command and checkpoint path, and your student ID (typed in the
  terminal is fine).
* The slider values (the command) are readable in every frame.
* Show, in this order, without the robot falling or being reset:

| # | Segment | Command | Duration |
|---|---|---|---|
| 1 | Stand and balance | $v_x=0$, $\omega_z=0$ | ≥ 10 s |
| 2 | Forward, then backward | $v_x=+0.8$, then $-0.8$ m/s | ≥ 5 s each |
| 3 | Turn in place, left then right | $\omega_z=+1.2$, then $-1.2$ rad/s | ≥ 5 s each |
| 4 | Drive and turn together | $v_x=0.6$, $\omega_z=0.8$ | ≥ 5 s |
| 5 | Stop | back to $0, 0$; the robot stops and stays up | ≥ 5 s |

If your command ranges are smaller, use your maximum values and say so in the email.

## Grading (100 points)

| Item | Points |
|---|---:|
| 1. Standing balance: stays up, little drift | 20 |
| 2. Forward/backward tracking: moves in the commanded direction at about the commanded speed | 25 |
| 3. Turning: turns in the commanded direction at about the commanded rate | 25 |
| 4. Combined command and stop | 15 |
| Video completeness: continuous, command readable, checkpoint path and student ID shown | 15 |

A fall or reset scores zero for the segment in which it happens. A video that is cut, sped up,
or does not show the command loses the completeness points.

## Submission

Email the MP4 (or a shareable link if it exceeds the attachment limit) to
morrisx28@mail.ntut.edu.tw with your name and student number in the subject, e.g.
`HW2 112xxxxxx Name`. Only the video is graded. Keep your `env_cfgs.py` and checkpoint until grades are out,
because we may ask you to run it again.
