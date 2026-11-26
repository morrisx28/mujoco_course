"""NTUT MuJoCo course: Pineapple V0 robot constants for mjlab v1.6.0 (given, do not edit).

Pineapple V0 is a wheeled biped. Each leg has a thigh and a calf joint (position controlled)
and a wheel joint (velocity controlled). There is no hip joint, so the robot cannot move
sideways: it drives forward/backward with its wheels and turns by spinning them differentially.

The MJCF is the course model ``robot/pineapple_v0/bipedwheel.xml``, copied unchanged into
``xmls/`` by ``hw2/starter/install_into_mjlab.py``. ``get_spec`` adapts it for mjlab:

1. adds a free joint to ``base_link`` (it is commented out in the course XML);
2. removes the XML ``<motor>`` actuators and ``<sensor>`` block, because mjlab creates its own
   actuators from ``PINEAPPLE_V0_ARTICULATION`` below and its own sensors from the task;
3. names the collision geoms so tasks can refer to them (contacts, friction randomization);
4. sets every geom margin to 0 (the XML default is 0.001), because MuJoCo Warp does not support
   a non-zero margin together with multi-point convex collision (MULTICCD).
"""

from pathlib import Path

import mujoco

from mjlab import MJLAB_SRC_PATH
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

##
# MJCF and assets.
##

PINEAPPLE_V0_XML: Path = (
  MJLAB_SRC_PATH / "asset_zoo" / "robots" / "pineapple_v0" / "xmls" / "bipedwheel.xml"
)
assert PINEAPPLE_V0_XML.exists(), "run hw2/starter/install_into_mjlab.py first"

BASE_BODY_NAME = "base_link"
LEG_JOINT_NAMES = ("L_thigh_joint", "L_calf_joint", "R_thigh_joint", "R_calf_joint")
WHEEL_JOINT_NAMES = ("L_wheel_joint", "R_wheel_joint")
# Bodies that must never touch the ground (everything except the wheels).
NON_WHEEL_BODY_NAMES = (
  "base_link",
  "L_thigh_link",
  "L_calf_link",
  "R_thigh_link",
  "R_calf_link",
)
WHEEL_GEOM_NAMES = ("L_wheel_collision", "R_wheel_collision")
WHEEL_RADIUS = 0.07  # m


def _name_collision_geoms(spec: mujoco.MjSpec) -> None:
  """Give every unnamed collision geom a name: <body>_collision, <body>_collision1, ..."""
  for body in spec.bodies:
    count = 0
    for geom in body.geoms:
      if geom.name or (geom.contype == 0 and geom.conaffinity == 0):
        continue  # already named, or visual only
      stem = body.name.removesuffix("_link")
      geom.name = f"{stem}_collision" + (str(count) if count else "")
      count += 1


def get_spec() -> mujoco.MjSpec:
  spec = mujoco.MjSpec.from_file(str(PINEAPPLE_V0_XML))
  base = spec.body(BASE_BODY_NAME)
  base.pos = [0.0, 0.0, 0.0]  # the initial state below sets the world position
  base.add_freejoint()
  for actuator in list(spec.actuators):
    spec.delete(actuator)
  for sensor in list(spec.sensors):
    spec.delete(sensor)
  _name_collision_geoms(spec)
  for geom in spec.geoms:
    geom.margin = 0.0
  return spec


##
# Actuators.
#
# Legs: joint-space PD position servos, limited to the XML motor range of 23.7 Nm.
#   Kp = 40 Nm/rad, Kd = 1 Nm s/rad hold the initial pose against gravity: the calves sag
#   about 0.06 rad when standing, 0.1 rad at the landing impact (see check_setup.py).
# Wheels: velocity servos (pure damping toward a target speed), limited to the XML
#   actuatorfrcrange of 3.69 Nm.
##

LEG_ACTUATOR = BuiltinPositionActuatorCfg(
  target_names_expr=(".*_thigh_joint", ".*_calf_joint"),
  stiffness=40.0,
  damping=1.0,
  effort_limit=23.7,
)
WHEEL_ACTUATOR = BuiltinVelocityActuatorCfg(
  target_names_expr=(".*_wheel_joint",),
  damping=0.5,
  effort_limit=3.69,
)

##
# Initial state.
#
# The pose of the (commented) keyframe in the course XML. With thigh 1.22 rad and calf
# -1.92 rad the wheel axles lie under the centre of mass (8 mm offset), and the wheels touch
# the ground when the base origin is 0.221 m high; 0.23 m leaves a small drop.
##

INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.23),
  joint_pos={
    ".*_thigh_joint": 1.22,
    ".*_calf_joint": -1.92,
    ".*_wheel_joint": 0.0,
  },
  joint_vel={".*": 0.0},
)

##
# Final config.
##

PINEAPPLE_V0_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(LEG_ACTUATOR, WHEEL_ACTUATOR),
  soft_joint_pos_limit_factor=0.9,
)


def get_pineapple_v0_robot_cfg() -> EntityCfg:
  """Return a fresh Pineapple V0 robot configuration."""
  return EntityCfg(
    init_state=INIT_STATE,
    spec_fn=get_spec,
    articulation=PINEAPPLE_V0_ARTICULATION,
  )


# Suggested action scales (you may change them in your task):
# legs: target = default pose + 0.5 * action (rad); wheels: target speed = 10 * action (rad/s).
PINEAPPLE_V0_LEG_ACTION_SCALE = 0.5
PINEAPPLE_V0_WHEEL_ACTION_SCALE = 10.0
PINEAPPLE_V0_WHEEL_VEL_LIMIT = 30.0  # rad/s, about 2.1 m/s at the rim


if __name__ == "__main__":
  import mujoco.viewer as viewer

  from mjlab.entity.entity import Entity

  robot = Entity(get_pineapple_v0_robot_cfg())
  viewer.launch(robot.spec.compile())
