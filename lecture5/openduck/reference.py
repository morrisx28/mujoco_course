"""Headless generation and independently inspectable kinematic diagnostics."""
from dataclasses import dataclass, asdict
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

ROBOT_DIR = Path(__file__).resolve().parent / 'robots' / 'open_duck_mini_v2'


@dataclass(frozen=True)
class WalkConfig:
    dx: float = 0.02                   # metres PER STEP, not m/s
    dy: float = 0.0
    dtheta: float = 0.0                # radians PER STEP
    duration: float = 6.0
    warmup: float = 2.0
    dt: float = 0.01
    fps: int = 50
    com_height: float = 0.20
    foot_height: float = 0.04
    single_support: float = 0.17
    double_support_ratio: float = 0.18


def _integer_ratio(value, name):
    result = round(value)
    if result < 1 or not np.isclose(value, result, rtol=0, atol=1e-9):
        raise ValueError(f'{name} must be a positive integer; got {value}')
    return result


def support_halfspaces(polygon):
    """Given polygon vertices (unordered allowed), return A,b for A @ xy <= b.

    Rows have unit outward normals, so b - A @ xy is a signed distance in m.
    """
    vertices = np.asarray(polygon, dtype=float)[:, :2]
    hull = ConvexHull(vertices)
    return hull.equations[:, :2], -hull.equations[:, 2]


def support_margin(point, polygon):
    A, b = support_halfspaces(polygon)
    return float(np.min(b - A @ np.asarray(point)[:2]))


def joint_bounds(names):
    nodes = {j.get('name'): j for j in ET.parse(ROBOT_DIR / 'open_duck_mini_v2.urdf').findall('joint')}
    bounds = []
    for name in names:
        node = nodes[name]
        limit = node.find('limit')
        bounds.append((-np.inf, np.inf) if node.get('type') == 'continuous' else
                      (float(limit.get('lower')), float(limit.get('upper'))))
    return np.asarray(bounds)


def generate_reference(config=None):
    """Return a sampled dict; arrays are time-major, in SI units.

    time starts at zero after warmup. engine_time retains the planning timestamp.
    root_T: (N,4,4), foot_T/planned_foot_T: (N,2,4,4), all world transforms.
    com/planned_com/planned_acceleration: (N,3); zmp: (N,2); contacts: (N,2), planned.
    joints: (N,J), ordered by joint_names; polygons is a list of world xy arrays.
    A failure propagates; no cached recording silently replaces generation.
    """
    config = WalkConfig() if config is None else config
    if not isinstance(config, WalkConfig):
        raise TypeError('config must be a WalkConfig')
    values = asdict(config)
    if not all(np.isfinite(v) for v in values.values()):
        raise ValueError('All walking parameters must be finite')
    if min(config.dt, config.fps, config.duration, config.com_height,
           config.single_support) <= 0 or min(config.warmup, config.foot_height,
                                              config.double_support_ratio) < 0:
        raise ValueError('Invalid time, height, or support parameter')
    stride = _integer_ratio(1 / (config.fps * config.dt), 'sample interval / dt')
    n = _integer_ratio(config.duration * config.fps, 'duration * fps')
    if n < 3:
        raise ValueError('At least three output samples are required')
    warmup = round(config.warmup / config.dt)
    if not np.isclose(warmup * config.dt, config.warmup):
        raise ValueError('warmup must be an integer multiple of dt')
    params = json.loads((ROBOT_DIR / 'placo_defaults.json').read_text())
    if not (-params['walk_max_dx_backward'] <= config.dx <= params['walk_max_dx_forward']
            and abs(config.dy) <= params['walk_max_dy']
            and abs(config.dtheta) <= params['walk_max_dtheta']):
        raise ValueError('Command exceeds preset step limits; refusing silent clipping')
    params.update(walk_com_height=config.com_height, walk_foot_height=config.foot_height,
                  single_support_duration=config.single_support,
                  double_support_ratio=config.double_support_ratio)
    from .engine import PlacoWalkEngine       # only generation needs Placo
    engine = PlacoWalkEngine(ROBOT_DIR / 'open_duck_mini_v2.urdf', params,
                             (config.dx, config.dy, config.dtheta), config.dt)
    for _ in range(warmup):
        engine.tick()
    keys = ('engine_time', 'root_T', 'foot_T', 'planned_foot_T', 'com', 'planned_com',
            'planned_acceleration', 'zmp', 'joints', 'contacts')
    out = {k: [] for k in keys}
    polygons = []
    omega = np.sqrt(9.81 / config.com_height)
    for index in range(n):
        if index:
            for _ in range(stride):
                engine.tick()
        t, tr, robot = engine.t, engine.trajectory, engine.robot
        row = (t, robot.get_T_world_fbase(),
               [robot.get_T_world_left(), robot.get_T_world_right()],
               [tr.get_T_world_left(t), tr.get_T_world_right(t)], robot.com_world(),
               tr.get_p_world_CoM(t), tr.get_a_world_CoM(t), tr.get_p_world_ZMP(t, omega),
               [robot.get_joint(j) for j in engine.joints], engine.contacts())
        for key, value in zip(keys, row):
            out[key].append(np.array(value, copy=True))
        polygons.append(np.asarray(tr.get_support(t).support_polygon(), dtype=float)[:, :2].copy())
    out = {k: np.asarray(v) for k, v in out.items()}
    if not all(np.isfinite(a).all() for a in out.values()):
        raise RuntimeError('Generator returned non-finite reference data')
    quat = Rotation.from_matrix(out['root_T'][:, :3, :3]).as_quat()  # xyzw
    for i in range(1, n):
        if np.dot(quat[i-1], quat[i]) < 0:
            quat[i] *= -1
    out.update(time=np.arange(n) / config.fps, root_quat=quat, polygons=polygons,
               joint_names=engine.joints, joint_bounds=joint_bounds(engine.joints),
               config=values, period=engine.period, gravity=9.81)
    return out


def diagnostics(samples):
    s = samples
    bounds, q = s['joint_bounds'], s['joints']
    joint_margin = np.minimum(q - bounds[:, 0], bounds[:, 1] - q).min(axis=0)
    foot_error = np.linalg.norm(s['foot_T'][:, :, :3, 3] - s['planned_foot_T'][:, :, :3, 3], axis=2)
    com_error = np.linalg.norm(s['com'] - s['planned_com'], axis=1)
    support = np.array([support_margin(p, poly) for p, poly in zip(s['zmp'], s['polygons'])])
    return dict(max_com_error_m=float(com_error.max()), max_foot_error_m=float(foot_error.max()),
                min_zmp_margin_m=float(support.min()),
                zmp_outside_samples=int(np.sum(support < -1e-9)),
                joint_violations_rad={j: float(-m) for j, m in zip(s['joint_names'], joint_margin) if m < -1e-8},
                max_joint_speed_rad_s=float(np.abs(np.diff(q, axis=0) * s['config']['fps']).max()),
                physics_verified=False)


def print_diagnostics(samples):
    report = diagnostics(samples)
    for key, value in report.items():
        print(f'{key}: {value}')
    if report['joint_violations_rad']:
        print('URDF JOINT LIMITS VIOLATED: this reference is not hardware-ready.')
    if report['zmp_outside_samples']:
        print('PLANNED ZMP OUTSIDE SUPPORT at some output samples: inspect the signed margins.')
    print('Contacts are scheduled labels. Torques, friction, collisions and balance are not verified.')
    return report
