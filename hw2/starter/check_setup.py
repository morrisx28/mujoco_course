"""Check the HW2 installation. Run from your mjlab checkout:

    uv run python /path/to/mujoco_course/hw2/starter/check_setup.py

1. The Pineapple V0 robot compiles in mjlab and has the expected joints and actuators.
2. With the legs servoed to the initial pose and the wheels braked (zero action), the legs
   hold the pose but the robot tips over within about a second. It is a wheeled inverted
   pendulum, so balancing it is your policy's job.
3. Whether your task is registered yet, and whether its config builds.
"""

import mujoco
import numpy as np

from mjlab.asset_zoo.robots.pineapple_v0.pineapple_v0_constants import (
  LEG_JOINT_NAMES,
  WHEEL_JOINT_NAMES,
  get_pineapple_v0_robot_cfg,
)
from mjlab.entity import Entity

TASK_ID = "Mjlab-Velocity-Flat-Pineapple-V0"


def tilt_deg(quat_wxyz: np.ndarray) -> float:
  w, x, y, z = quat_wxyz
  up_z = 1 - 2 * (x * x + y * y)  # z component of the body z axis in the world
  return float(np.degrees(np.arccos(np.clip(up_z, -1, 1))))


def main() -> None:
  # 1. Robot model.
  spec = Entity(get_pineapple_v0_robot_cfg()).spec
  spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05])
  model = spec.compile()
  joints = [model.joint(i).name for i in range(model.njnt) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE]
  actuators = [model.actuator(i).name for i in range(model.nu)]
  assert sorted(joints) == sorted(LEG_JOINT_NAMES + WHEEL_JOINT_NAMES), joints
  assert sorted(actuators) == sorted(joints), actuators
  print(f"[ok] robot compiles: total mass {model.body_subtreemass[1]:.2f} kg")
  print(f"     joints/actuators: {', '.join(joints)}")

  # 2. Passive test with the initial-state keyframe (legs at default pose, wheels braked).
  data = mujoco.MjData(model)
  mujoco.mj_resetDataKeyframe(model, data, 0)
  default = data.qpos[7:].copy()
  fall_time, max_leg_err = None, 0.0
  for _ in range(int(2.0 / model.opt.timestep)):
    mujoco.mj_step(model, data)
    if data.time < 0.5:
      leg_idx = [model.joint(n).qposadr[0] - 7 for n in LEG_JOINT_NAMES]
      max_leg_err = max(max_leg_err, float(np.max(np.abs(data.qpos[7:][leg_idx] - default[leg_idx]))))
    if fall_time is None and tilt_deg(data.qpos[3:7]) > 30:
      fall_time = data.time
  assert max_leg_err < 0.15, f"legs sag {max_leg_err:.3f} rad: the leg servos cannot hold the pose"
  print(f"[ok] leg servos hold the initial pose (max error {max_leg_err:.3f} rad in the first 0.5 s)")
  if fall_time is None:
    print("[??] the robot did not tip over with zero action; please report this to the TA")
  else:
    print(f"[ok] with zero action the robot tips past 30 deg after {fall_time:.2f} s: your policy must balance it")

  # 3. Task registration.
  import mjlab.tasks  # noqa: F401  (imports every task package)
  from mjlab.tasks.registry import list_tasks, load_env_cfg

  if TASK_ID not in list_tasks():
    print(f"[--] {TASK_ID} is not registered yet. Finish env_cfgs.py, then uncomment __init__.py.")
    return
  cfg = load_env_cfg(TASK_ID)
  print(f"[ok] {TASK_ID} is registered")
  for name in ("actions", "commands", "observations", "rewards", "terminations", "events"):
    print(f"     {name:13s}: {', '.join(getattr(cfg, name))}")
  assert "twist" in cfg.commands, 'name the velocity command "twist" (the play-mode joystick uses it)'
  print("Next: uv run play", TASK_ID, "--agent zero   (then --agent random, then train)")


if __name__ == "__main__":
  main()
