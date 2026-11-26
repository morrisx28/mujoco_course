"""HW2: Pineapple V0 flat-ground velocity tracking with mjlab.

Fill in the six ``_*`` functions marked TODO. ``pineapple_v0_flat_env_cfg`` at the bottom
(scene, contact sensor, simulation, viewer and play mode) is given: do not change it.

Useful references inside this mjlab checkout:
  src/mjlab/tasks/velocity/velocity_env_cfg.py   the generic velocity task (legged robots)
  src/mjlab/tasks/velocity/config/go1/env_cfgs.py how a robot customizes it
  src/mjlab/tasks/velocity/mdp/, src/mjlab/envs/mdp/  every observation/reward/termination
When you finish, uncomment the registration in __init__.py.
"""

from __future__ import annotations

import math

from mjlab.asset_zoo.robots.pineapple_v0.pineapple_v0_constants import (
  BASE_BODY_NAME,
  LEG_JOINT_NAMES,
  NON_WHEEL_BODY_NAMES,
  PINEAPPLE_V0_LEG_ACTION_SCALE,
  PINEAPPLE_V0_WHEEL_ACTION_SCALE,
  PINEAPPLE_V0_WHEEL_VEL_LIMIT,
  WHEEL_GEOM_NAMES,
  WHEEL_JOINT_NAMES,
  get_pineapple_v0_robot_cfg,
)
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg, JointVelocityActionCfg
from mjlab.managers.action_manager import ActionTermCfg
from mjlab.managers.command_manager import CommandTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.scene import SceneCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.velocity import mdp
from mjlab.tasks.velocity.mdp import UniformVelocityCommandCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise
from mjlab.viewer import ViewerConfig

LEGS = SceneEntityCfg("robot", joint_names=LEG_JOINT_NAMES)
WHEELS = SceneEntityCfg("robot", joint_names=WHEEL_JOINT_NAMES)

# Given: contact between any non-wheel body and the ground. Use it in rewards/terminations
# as ``sensor_name="body_ground_contact"``.
BODY_GROUND_CONTACT = ContactSensorCfg(
  name="body_ground_contact",
  primary=ContactMatch(mode="body", pattern=NON_WHEEL_BODY_NAMES, entity="robot"),
  secondary=ContactMatch(mode="body", pattern="terrain"),
  fields=("found", "force"),
  reduce="none",
  num_slots=1,
  history_length=4,
)


##
# Student part. Every function returns a dict {term name: term config}.
##


def _actions() -> dict[str, ActionTermCfg]:
  """TODO: two action terms.

  * legs: ``JointPositionActionCfg`` on ``LEG_JOINT_NAMES``, around the default pose
    (``use_default_offset=True``), scale ``PINEAPPLE_V0_LEG_ACTION_SCALE``.
  * wheels: ``JointVelocityActionCfg`` on ``WHEEL_JOINT_NAMES``, scale
    ``PINEAPPLE_V0_WHEEL_ACTION_SCALE``; clip the target to ``PINEAPPLE_V0_WHEEL_VEL_LIMIT``.
  """
  raise NotImplementedError("_actions")


def _commands() -> dict[str, CommandTermCfg]:
  """TODO: one command named "twist" (the name is used by the play-mode joystick).

  Use ``UniformVelocityCommandCfg`` with ``heading_command=False``. The robot has no hip
  joint, so keep ``lin_vel_y=(0.0, 0.0)``. Suggested ranges: ``lin_vel_x`` +-1.0 m/s,
  ``ang_vel_z`` +-1.5 rad/s, with some standing envs (``rel_standing_envs``). The ranges
  are also the slider limits in play mode.
  """
  raise NotImplementedError("_commands")


def _observations() -> dict[str, ObservationGroupCfg]:
  """TODO: two groups, "actor" (policy input) and "critic" (value function input).

  The actor needs what a real robot can measure: base angular velocity, projected gravity,
  the command, leg joint positions, joint velocities and the last action. Add sensor
  noise with ``noise=Unoise(...)`` and ``enable_corruption=True``. Wheel angles grow
  without bound, so observe joint positions of the legs only (``params={"asset_cfg": LEGS}``).
  The critic may also see privileged data such as ``mdp.base_lin_vel``.
  """
  raise NotImplementedError("_observations")


def _rewards() -> dict[str, RewardTermCfg]:
  """TODO: rewards for following the command and staying balanced.

  * tracking: ``mdp.track_linear_velocity`` and ``mdp.track_angular_velocity``
    (``params={"command_name": "twist", "std": ...}``), positive weights;
  * balance: e.g. ``mdp.flat_orientation_l2`` (negative weight) and a penalty for falling
    (``mdp.is_terminated``);
  * posture and effort: e.g. ``mdp.posture`` on the legs, ``mdp.action_rate_l2``,
    ``mdp.joint_torques_l2``, ``mdp.joint_pos_limits``.
  Start small, check the reward curves in TensorBoard, then tune the weights.
  """
  raise NotImplementedError("_rewards")


def _terminations() -> dict[str, TerminationTermCfg]:
  """TODO: end an episode on time-out or failure.

  ``mdp.time_out`` (with ``time_out=True``), ``mdp.bad_orientation`` (a tilt limit), and
  ``mdp.illegal_contact`` with ``sensor_name="body_ground_contact"`` (any body other
  than a wheel touches the ground).
  """
  raise NotImplementedError("_terminations")


def _events() -> dict[str, EventTermCfg]:
  """TODO: resets and randomization.

  At least reset the base (``mdp.reset_root_state_uniform``) and the leg joints
  (``mdp.reset_joints_by_offset``) with some randomness. Optional but recommended for a
  robust policy: random pushes (``mdp.push_by_setting_velocity``, interval mode), wheel
  friction (``dr.geom_friction`` on ``WHEEL_GEOM_NAMES``) and base mass (``dr.body_mass``).
  Play mode removes the "push_robot" event and zeroes the "reset_base" ranges, so use
  those two names if you add them.
  """
  raise NotImplementedError("_events")


##
# Given: assemble the environment.
##


def pineapple_v0_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Pineapple V0 velocity tracking on flat ground."""
  cfg = ManagerBasedRlEnvCfg(
    scene=SceneCfg(
      terrain=TerrainEntityCfg(terrain_type="plane"),
      entities={"robot": get_pineapple_v0_robot_cfg()},
      sensors=(BODY_GROUND_CONTACT,),
      num_envs=1,
      env_spacing=2.0,
    ),
    observations=_observations(),
    actions=_actions(),
    commands=_commands(),
    events=_events(),
    rewards=_rewards(),
    terminations=_terminations(),
    curriculum={},
    viewer=ViewerConfig(
      origin_type=ViewerConfig.OriginType.ASSET_BODY,
      entity_name="robot",
      body_name=BASE_BODY_NAME,
      distance=2.0,
      elevation=-10.0,
      azimuth=90.0,
    ),
    sim=SimulationCfg(
      njmax=300,
      nconmax=None,
      contact_sensor_maxmatch=64,
      mujoco=MujocoCfg(timestep=0.005, iterations=10, ls_iterations=20),
    ),
    decimation=4,  # policy at 50 Hz, physics at 200 Hz
    episode_length_s=20.0,
  )

  if play:
    cfg.episode_length_s = int(1e9)  # never time out while you film
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    if "reset_base" in cfg.events:  # start upright, in place, at rest
      cfg.events["reset_base"].params["pose_range"] = {}
      cfg.events["reset_base"].params["velocity_range"] = {}
  return cfg
