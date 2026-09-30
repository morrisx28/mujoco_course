"""Optional finite MeshCat animation and always-available Matplotlib diagnostics."""
import numpy as np
from .reference import ROBOT_DIR, support_margin
from .motion import validate_episode, frame_slices


def plot_reference(samples):
    import matplotlib.pyplot as plt
    s, t = samples, samples['time']
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    ax = axes[0, 0]
    ax.step(t, s['contacts'][:, 0], where='post', label='left support')
    ax.step(t, s['contacts'][:, 1] + 1.2, where='post', label='right support (+1.2)')
    ax.set(xlabel='Time (s)', ylabel='Scheduled contact', title='Contact schedule (not force measurements)')
    ax = axes[0, 1]
    for polygon in s['polygons'][::max(1, len(t)//20)]:
        from scipy.spatial import ConvexHull
        v = polygon[ConvexHull(polygon).vertices]
        ax.fill(v[:, 0], v[:, 1], alpha=0.08, color='gray')
    ax.plot(*s['planned_com'][:, :2].T, label='planned CoM')
    ax.plot(*s['zmp'][:, :2].T, label='planned ZMP')
    outside = np.array([support_margin(p, poly) < -1e-9
                        for p, poly in zip(s['zmp'], s['polygons'])])
    if outside.any():
        ax.scatter(*s['zmp'][outside, :2].T, color='red', marker='x',
                   s=35, zorder=5, label='outside active support')
    for side, name in enumerate(('left', 'right')):
        ax.plot(*s['planned_foot_T'][:, side, :2, 3].T, '--', label=f'{name} foot')
    ax.set(xlabel='World x (m)', ylabel='World y (m)', title='Footpaths and support polygons')
    ax.axis('equal')
    ax = axes[1, 0]
    for side, name in enumerate(('left', 'right')):
        ax.plot(t, s['planned_foot_T'][:, side, 2, 3], label=f'{name} planned')
        ax.plot(t, s['foot_T'][:, side, 2, 3], '--', label=f'{name} achieved')
    ax.set(xlabel='Time (s)', ylabel='Foot height (m)', title='Swing clearance and tracking')
    ax = axes[1, 1]
    for dim, name in enumerate(('x', 'y')):
        ax.plot(t, s['planned_com'][:, dim], label=f'CoM {name} planned')
        ax.plot(t, s['com'][:, dim], '--', label=f'CoM {name} achieved')
        ax.plot(t, s['zmp'][:, dim], ':', label=f'ZMP {name}')
    ax.set(xlabel='Time (s)', ylabel='Position (m)', title='CoM / ZMP')
    for ax in axes.flat:
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.tight_layout()
    return fig


def plot_joint_motion(samples, velocity):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for i, name in enumerate(samples['joint_names']):
        if name.startswith(('left_', 'right_')) and 'antenna' not in name:
            axes[0].plot(samples['time'], samples['joints'][:, i], label=name)
            axes[1].plot(samples['time'], velocity[:, i], label=name)
    axes[0].set(ylabel='Joint angle (rad)', title='Generated joint motion (limits reported separately)')
    axes[1].set(xlabel='Time (s)', ylabel='Joint velocity (rad/s)')
    axes[0].legend(ncol=2, fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    return fig


def replay_episode(episode):
    """Return an embedded-viewer object; finite animation, no sleep/while loop.

    Import/launch failures may be handled by the notebook. Malformed data,
    mesh loading, joint mapping and animation failures deliberately propagate.
    """
    validate_episode(episode)
    import meshcat
    import meshcat.geometry as geometry
    from meshcat.animation import Animation
    import placo
    from pinocchio.visualize import MeshcatVisualizer
    from scipy.spatial.transform import Rotation
    robot = placo.RobotWrapper(str(ROBOT_DIR / 'open_duck_mini_v2.urdf'))
    missing = set(episode['Joints']) - set(robot.joint_names())
    if missing:
        raise ValueError(f'Unknown robot joints: {sorted(missing)}')
    viewer = meshcat.Visualizer()
    viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    viz.initViewer(viewer=viewer)
    viz.loadViewerModel('open_duck')
    for side in ('left', 'right'):
        for phase, color in (('support', 0x22BB55), ('swing', 0xEE5533)):
            viewer[f'{side}_{phase}'].set_object(geometry.Sphere(0.008),
                                                geometry.MeshLambertMaterial(color=color))
    frames, slices = np.asarray(episode['Frames']), frame_slices(episode)
    animation = Animation(default_framerate=episode['FPS'])
    for index, row in enumerate(frames):
        q = robot.state.q.copy()
        q[:3], q[3:7] = row[slices['root_pos']], row[slices['root_quat']]
        for name, angle in zip(episode['Joints'], row[slices['joints_pos']]):
            q[robot.get_joint_offset(name)] = angle
        with animation.at_frame(viewer, index) as frame:
            viz.viewer = frame
            viz.display(q)
            R = Rotation.from_quat(q[3:7]).as_matrix()
            for j, side in enumerate(('left', 'right')):
                T = np.eye(4)
                T[:3, 3] = q[:3] + R @ row[slices[f'{side}_toe_pos']]
                contact = bool(row[slices['foot_contacts']][j])
                for phase, visible in (('support', contact), ('swing', not contact)):
                    frame[f'{side}_{phase}'].set_transform(T)
                    frame[f'{side}_{phase}'].set_property('visible', 'bool', visible)
    viz.viewer = viewer
    viewer.set_animation(animation, play=True, repetitions=1)
    return viewer
