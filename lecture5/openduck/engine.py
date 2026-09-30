"""Course adaptation of Open Duck's PlacoWalkEngine; see PROVENANCE.md.

This is a kinematic reference generator, not a physics simulator or controller.
Only the dependency adapter is here; student mathematics stays in the notebooks.
"""
import numpy as np
import placo


def _make_supports(footsteps, middle):
    # Preserve the local source's compatibility with the older 0.6 API.
    try:
        return placo.FootstepsPlanner.make_supports(footsteps, 0.0, True, middle, True)
    except TypeError:
        return placo.FootstepsPlanner.make_supports(footsteps, True, middle, True)


class PlacoWalkEngine:
    def __init__(self, urdf, parameters, command, dt=0.01, refine=10):
        self.robot = placo.HumanoidRobot(str(urdf))
        self.parameters = placo.HumanoidParameters()
        for key, value in parameters.items():
            if hasattr(self.parameters, key):
                setattr(self.parameters, key,
                        np.deg2rad(value) if key == 'walk_trunk_pitch' else value)
        self.replan_timesteps = parameters.get('replan_timesteps', 10)
        self.joints = list(parameters['joints'])
        self.dt, self.refine, self.t, self.last_replan = dt, refine, 0.0, 0.0
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.enable_velocity_limits(True)
        self.robot.set_velocity_limits(12.0)
        # Match the reference generator, and REPORT URDF limit violations separately.
        # Do not copy its reversed knee limit overrides; retain the original URDF.
        self.solver.enable_joint_limits(False)
        self.solver.dt = dt / refine
        self.tasks = placo.WalkTasks()
        if hasattr(self.parameters, 'trunk_mode'):
            self.tasks.trunk_mode = self.parameters.trunk_mode
        self.tasks.com_x = 0.0
        self.tasks.initialize_tasks(self.solver, self.robot)
        self.tasks.left_foot_task.orientation().mask.set_axises('yz', 'local')
        self.tasks.right_foot_task.orientation().mask.set_axises('yz', 'local')
        self.posture = self.solver.add_joints_task()
        self.posture.set_joints({k: np.deg2rad(v) for k, v in parameters['joint_angles'].items()})
        self.posture.configure('joints', 'soft', 1.0)
        self.tasks.reach_initial_pose(np.eye(4), self.parameters.feet_spacing,
                                      self.parameters.walk_com_height,
                                      self.parameters.walk_trunk_pitch)
        self.planner = placo.FootstepsPlannerRepetitive(self.parameters)
        self.planner.configure(*command, 5)
        left = placo.flatten_on_floor(self.robot.get_T_world_left())
        right = placo.flatten_on_floor(self.robot.get_T_world_right())
        footsteps = self.planner.plan(placo.HumanoidRobot_Side.left, left, right)
        supports = _make_supports(footsteps, self.parameters.has_double_support())
        self.walk = placo.WalkPatternGenerator(self.robot, self.parameters)
        self.trajectory = self.walk.plan(supports, self.robot.com_world(), 0.0)
        self.period = 2 * (self.parameters.single_support_duration
                           + self.parameters.double_support_duration())

    def tick(self):
        # Every solved state and sampled reference share the SAME timestamp.
        for k in range(self.refine):
            target_time = self.t + (k + 1) * self.dt / self.refine
            self.tasks.update_tasks_from_trajectory(self.trajectory, target_time)
            self.robot.update_kinematics()
            self.solver.solve(True)
        self.t += self.dt
        self.robot.update_kinematics()
        if (self.t - self.last_replan > self.replan_timesteps * self.parameters.dt()
                and self.walk.can_replan_supports(self.trajectory, self.t)):
            try:
                supports = self.walk.replan_supports(self.planner, self.trajectory,
                                                      self.t, self.last_replan)
            except TypeError:
                supports = self.walk.replan_supports(self.planner, self.trajectory, self.t)
            self.trajectory = self.walk.replan(supports, self.trajectory, self.t)
            self.last_replan = self.t
            if hasattr(self.trajectory, 'replan_success') and not self.trajectory.replan_success:
                raise RuntimeError('Placo could not replan the walking trajectory')

    def contacts(self):
        if self.trajectory.support_is_both(self.t):
            return [1, 1]
        side = self.trajectory.support_side(self.t)
        if side == placo.HumanoidRobot_Side.left:
            return [1, 0]
        if side == placo.HumanoidRobot_Side.right:
            return [0, 1]
        raise RuntimeError(f'Unknown support side: {side}')
