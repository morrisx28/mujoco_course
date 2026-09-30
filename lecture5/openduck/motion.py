"""Validate and persist the documented Open Duck course motion format.

No velocity computation or frame assembly is hidden here: those are student TODOs.
"""
import json
from pathlib import Path
import numpy as np

FIELDS = ('root_pos', 'root_quat', 'joints_pos', 'left_toe_pos', 'right_toe_pos',
          'world_linear_vel', 'world_angular_vel', 'joints_vel',
          'left_toe_vel', 'right_toe_vel', 'foot_contacts')
CONVENTIONS = dict(quaternion='xyzw', root_pose='world', linear_velocity='world',
                   angular_velocity='world', toe_positions='body',
                   toe_velocities='derivative of body coordinates', contacts='planned, not measured')


def frame_slices(episode):
    offsets, sizes = episode['Frame_offset'][0], episode['Frame_size'][0]
    return {name: slice(offsets[name], offsets[name] + sizes[name]) for name in offsets}


def validate_episode(episode):
    if episode.get('FormatVersion') != 'openduck-course-1' or episode.get('Robot') != 'open_duck_mini_v2':
        raise ValueError('Unsupported course format or robot')
    if episode.get('CoordinateConventions') != CONVENTIONS:
        raise ValueError('Missing or conflicting coordinate conventions')
    joints = episode['Joints']
    if (not isinstance(joints, list) or not joints
            or not all(isinstance(j, str) and j for j in joints)
            or len(set(joints)) != len(joints)):
        raise ValueError('Joint names must be nonempty and unique')
    if len(episode['Frame_offset']) != 1 or len(episode['Frame_size']) != 1:
        raise ValueError('Expected one frame layout')
    sizes, offsets = episode['Frame_size'][0], episode['Frame_offset'][0]
    expected = dict(zip(FIELDS, (3, 4, len(joints), 3, 3, 3, 3, len(joints), 3, 3, 2)))
    if (sizes != expected or set(offsets) != set(FIELDS)
            or not all(type(size) is int for size in sizes.values())):
        raise ValueError('Incorrect frame field sizes or names')
    end = 0
    for name in sorted(FIELDS, key=offsets.get):
        if type(offsets[name]) is not int or offsets[name] != end:
            raise ValueError('Frame fields must be contiguous and non-overlapping')
        end += sizes[name]
    frames = np.asarray(episode['Frames'], dtype=float)
    if frames.ndim != 2 or frames.shape[0] < 3 or frames.shape[1] != end or not np.isfinite(frames).all():
        raise ValueError('Invalid frame array')
    fps, dt = episode['FPS'], episode['FrameDuration']
    if not np.isfinite([fps, dt]).all() or fps <= 0 or dt <= 0 or not np.isclose(fps * dt, 1):
        raise ValueError('FPS and FrameDuration disagree')
    times = np.asarray(episode['Times'], dtype=float)
    if times.shape != (len(frames),) or not np.isfinite(times).all() or not np.isclose(times[0], 0) or not np.allclose(np.diff(times), dt):
        raise ValueError('Invalid sample timestamps')
    slices = frame_slices(episode)
    quat = frames[:, slices['root_quat']]
    if not np.allclose(np.linalg.norm(quat, axis=1), 1, atol=1e-7, rtol=0):
        raise ValueError('Quaternions must be normalized')
    if np.any(np.sum(quat[1:] * quat[:-1], axis=1) < 0):
        raise ValueError('Quaternion signs must be temporally continuous')
    contacts = frames[:, slices['foot_contacts']]
    if not np.isin(contacts, [0, 1]).all() or np.any(contacts.sum(axis=1) == 0):
        raise ValueError('Walking contacts must be left, right or double support')
    return episode


def save_episode(episode, path):
    validate_episode(episode)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(episode, indent=2, allow_nan=False) + '\n')
    return path


def load_episode(path):
    return validate_episode(json.loads(Path(path).read_text()))
