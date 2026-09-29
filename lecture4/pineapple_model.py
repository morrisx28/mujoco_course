"""pineapple_v0 model: MJCF parsing and symbolic kinematics. Given code, no TODOs.

Parses robot/pineapple_v0/bipedwheel.xml directly (no mujoco import) and builds a
symbolic static model with CasADi. Shared by lecture 3 (solving the standing
equilibrium with Newton's method) and lecture 4 (linearizing it for LQR).

q = [base_xyz, base_roll_pitch_yaw, L_thigh, L_calf, L_wheel,
     R_thigh, R_calf, R_wheel]; R = Rz(yaw) Ry(pitch) Rx(roll).
The body is modeled as free even if <freejoint/> is commented out in the MJCF.
Source XML files are never modified.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np


MODEL_PATH = Path(__file__).resolve().parents[1] / 'robot/pineapple_v0/bipedwheel.xml'
JOINT_NAMES = tuple(f'{side}_{part}_joint' for side in ('L', 'R')
                    for part in ('thigh', 'calf', 'wheel'))
DEFAULT_JOINT_POSITIONS = np.array([1.27, -2.127, 0.0, 1.27, -2.127, 0.0])
WHEEL_BODIES = ('L_wheel_link', 'R_wheel_link')


def vec(value):
    return np.asarray(value, dtype=float).reshape(-1)


def numbers(element, name, default):
    return np.fromstring(element.get(name, default), sep=' ')


def attribute_numbers(attributes, name, default):
    """Same as numbers() but for an already class-merged attribute dict."""
    return np.fromstring(attributes.get(name, default), sep=' ')


def rotation(axis, angle):
    axis = ca.DM(axis)
    axis /= ca.norm_2(axis)
    S = ca.skew(axis)
    return ca.SX.eye(3) + ca.sin(angle) * S + (1 - ca.cos(angle)) * S @ S


def quaternion_rotation(quat):
    w, x, y, z = np.asarray(quat) / np.linalg.norm(quat)
    return ca.DM([[1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                  [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                  [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]])


def rpy_quaternion(rpy):
    roll, pitch, yaw = np.asarray(rpy) / 2
    cr, cp, cy = np.cos([roll, pitch, yaw])
    sr, sp, sy = np.sin([roll, pitch, yaw])
    return np.array([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy])


class PineappleModel:
    """Read the local MJCF tree directly and construct a symbolic static model."""

    def __init__(self, path=MODEL_PATH):
        self.path = Path(path).resolve()
        root = ET.parse(self.path).getroot()
        self.gravity = np.array([0.0, 0.0, -9.81])
        option = root.find('option')
        if option is not None and 'gravity' in option.attrib:
            self.gravity = numbers(option, 'gravity', '0 0 -9.81')
        if not np.allclose(self.gravity[:2], 0) or self.gravity[2] >= 0:
            raise ValueError('This flat-ground example requires downward vertical gravity')
        defaults = {}

        def read_defaults(node, inherited):
            values = {key: dict(value) for key, value in inherited.items()}
            for tag in ('joint', 'geom', 'motor'):
                element = node.find(tag)
                if element is not None:
                    values.setdefault(tag, {}).update(element.attrib)
            defaults[node.get('class', '')] = values
            for child in node.findall('default'):
                read_defaults(child, values)

        if root.find('default') is not None:
            read_defaults(root.find('default'), {})

        def attributes(element, tag, inherited_class):
            return {**defaults.get(element.get('class', inherited_class), {}).get(tag, {}),
                    **element.attrib}

        base = root.find('worldbody/body')
        if base is None or base.get('name') != 'base_link':
            raise ValueError('Expected pineapple_v0 base_link as the root body')
        q = ca.SX.sym('q', 12)
        Rbase = rotation([0, 0, 1], q[5]) @ rotation([0, 1, 0], q[4]) @ rotation([1, 0, 0], q[3])
        bodies, centers, masses, wheel_data, visuals, inertials = {}, [], [], {}, [], {}
        self.joint_limits = {}

        def visit(body, Rp, pp, inherited_class, is_base=False):
            child_class = body.get('childclass', inherited_class)
            if is_base:
                R, p = Rbase, q[:3]
            else:
                Rlocal = quaternion_rotation(numbers(body, 'quat', '1 0 0 0'))
                R, p = Rp @ Rlocal, pp + Rp @ numbers(body, 'pos', '0 0 0')
            for joint in body.findall('joint'):
                a = attributes(joint, 'joint', child_class)
                name = a.get('name')
                if name not in JOINT_NAMES or a.get('type', 'hinge') != 'hinge':
                    raise ValueError(f'Unsupported joint: {name}')
                axis = np.fromstring(a.get('axis', '0 0 1'), sep=' ')
                pivot = np.fromstring(a.get('pos', '0 0 0'), sep=' ')
                Rjoint = rotation(axis, q[6 + JOINT_NAMES.index(name)])
                p = p + R @ (pivot - Rjoint @ pivot)
                R = R @ Rjoint
                if 'range' in a:
                    self.joint_limits[name] = np.fromstring(a['range'], sep=' ')
                if name.endswith('wheel_joint'):
                    wheel_data[name[0]] = {'axis': R @ ca.DM(axis / np.linalg.norm(axis))}
            name = body.get('name')
            bodies[name] = (R, p)
            # Visual meshes for MeshCat. group 1 mesh geoms are the render shapes;
            # the collision primitives (boxes, cylinders) are deliberately skipped.
            for geom in body.findall('geom'):
                a = attributes(geom, 'geom', child_class)
                if a.get('type') != 'mesh' or 'mesh' not in a:
                    continue
                local = np.eye(4)
                local[:3, :3] = np.asarray(quaternion_rotation(
                    attribute_numbers(a, 'quat', '1 0 0 0')))
                local[:3, 3] = attribute_numbers(a, 'pos', '0 0 0')
                visuals.append({'body': name, 'mesh': a['mesh'], 'transform': local,
                                'rgba': attribute_numbers(a, 'rgba', '0.5 0.5 0.5 1')})
            inertial = body.find('inertial')
            if inertial is not None:
                masses.append(float(inertial.get('mass')))
                centers.append(p + R @ numbers(inertial, 'pos', '0 0 0'))
                # The equilibrium itself needs only masses and centers, but a
                # cart-pole reduction needs the rotational inertia too, so keep it.
                inertials[name] = {'mass': float(inertial.get('mass')),
                                   'pos': numbers(inertial, 'pos', '0 0 0'),
                                   'quat': numbers(inertial, 'quat', '1 0 0 0'),
                                   'diaginertia': numbers(inertial, 'diaginertia', '0 0 0')}
            if name in ('L_wheel_link', 'R_wheel_link'):
                geom = next(g for g in body.findall('geom') if g.get('class') == 'wheel')
                a = attributes(geom, 'geom', child_class)
                wheel_data[name[0]].update(
                    center=p + R @ numbers(geom, 'pos', '0 0 0'),
                    radius=float(a['size'].split()[0]),
                    friction=float(a['friction'].split()[0]))
            for child in body.findall('body'):
                visit(child, R, p, child_class)

        visit(base, Rbase, q[:3], '', is_base=True)
        if len(masses) != 8 or any(m <= 0 for m in masses):
            raise ValueError('Expected the eight positive-mass pineapple_v0 bodies')
        self.mass = sum(masses)
        self.inertials = inertials
        self.body_names = list(bodies)
        self.edges = [(b.get('name'), c.get('name')) for b in base.iter('body') for c in b.findall('body')]
        V = sum(-mass * ca.dot(self.gravity, center) for mass, center in zip(masses, centers))
        gravity = ca.gradient(V, q)
        com = sum(mass * center for mass, center in zip(masses, centers)) / self.mass
        contacts, contact_jacobians, wheel_centers = [], [], []
        self.radii, self.friction = [], []
        normal = ca.DM([0, 0, 1])
        for side in ('L', 'R'):
            wheel = wheel_data[side]
            axle, center, radius = wheel['axis'], wheel['center'], wheel['radius']
            projected_normal = normal - axle * ca.dot(axle, normal)
            offset = -radius * projected_normal / ca.norm_2(projected_normal)
            contacts.append(center + offset)
            wheel_centers.append(center)
            R = bodies[f'{side}_wheel_link'][0]
            columns = []
            for k in range(12):
                dR = ca.reshape(ca.jacobian(ca.reshape(R, 9, 1), q)[:, k], 3, 3)
                skew = dR @ R.T
                columns.append(ca.vertcat(skew[2, 1], skew[0, 2], skew[1, 0]))
            angular_jacobian = ca.horzcat(*columns)
            # Contact-force virtual work uses the instantaneous material point.
            # Differentiating the lowest-point locus would erase wheel torque.
            contact_jacobians.append(ca.jacobian(center, q) - ca.skew(offset) @ angular_jacobian)
            self.radii.append(radius)
            self.friction.append(wheel['friction'])
        J = ca.vertcat(*contact_jacobians)
        self.evaluate = ca.Function('static_model', [q], [gravity, J, ca.horzcat(*contacts),
                                                        com, ca.horzcat(*wheel_centers)])
        self.potential = ca.Function('potential', [q], [V])
        self.positions = ca.Function('body_positions', [q], [ca.horzcat(*[p for _, p in bodies.values()])])
        # 4x4 world transform per body, stacked horizontally: columns 4*i:4*i+4 are body i.
        # MeshCat needs orientation as well as position, which self.positions drops.
        self.transforms = ca.Function('body_transforms', [q], [ca.horzcat(
            *[ca.vertcat(ca.horzcat(R, p), ca.DM([[0, 0, 0, 1]])) for R, p in bodies.values()])])
        compiler = root.find('compiler')
        mesh_dir = compiler.get('meshdir', '') if compiler is not None else ''
        mesh_files = {mesh.get('name'): (self.path.parent / mesh_dir / mesh.get('file')).resolve()
                      for mesh in root.findall('asset/mesh')}
        for visual in visuals:
            path = mesh_files.get(visual['mesh'])
            if path is None or not path.is_file():
                raise FileNotFoundError(f"Missing visual mesh for geom '{visual['mesh']}': {path}")
            visual['mesh_path'] = path
        self.visuals = visuals
        self.torque_limits = np.full(6, np.inf)
        for motor in root.findall('actuator/motor'):
            a = attributes(motor, 'motor', '')
            index = JOINT_NAMES.index(a['joint'])
            if 'ctrlrange' in a:
                self.torque_limits[index] = max(abs(np.fromstring(a['ctrlrange'], sep=' ')))
        for joint in base.iter('joint'):
            if 'actuatorfrcrange' in joint.attrib:
                index = JOINT_NAMES.index(joint.get('name'))
                self.torque_limits[index] = min(self.torque_limits[index],
                                                max(abs(numbers(joint, 'actuatorfrcrange', '0 0'))))
        self.q_guess = np.r_[numbers(base, 'pos', '0 0 0'), np.zeros(3), DEFAULT_JOINT_POSITIONS]
        _, _, c0, _, _ = self.evaluate(self.q_guess)
        self.q_guess[2] -= np.asarray(c0)[2].mean()
        self.contact_midpoint_xy = np.asarray(self.evaluate(self.q_guess)[2])[:2].mean(axis=1)


    def composite_inertia(self, q, exclude=()):
        """Total mass, world center of mass, and 3x3 inertia about that center.

        `exclude` names bodies to leave out - pass WHEEL_BODIES to describe only the
        sprung body, whose wheels spin independently and are accounted for separately
        in a wheeled-inverted-pendulum reduction.

        Each body's <inertial> diaginertia is expressed in its own principal frame, so
        it is rotated into the world at this configuration and then shifted onto the
        combined center of mass with the parallel axis theorem.
        """
        transforms = np.asarray(self.transforms(q))
        total_mass, weighted, parts = 0.0, np.zeros(3), []
        for index, name in enumerate(self.body_names):
            if name in exclude or name not in self.inertials:
                continue
            data = self.inertials[name]
            T = transforms[:, 4 * index:4 * index + 4]
            center = T[:3, :3] @ data['pos'] + T[:3, 3]
            R = T[:3, :3] @ np.asarray(quaternion_rotation(data['quat']))
            parts.append((data['mass'], center, R @ np.diag(data['diaginertia']) @ R.T))
            total_mass += data['mass']
            weighted += data['mass'] * center
        if not parts:
            raise ValueError('composite_inertia: every body was excluded')
        com = weighted / total_mass
        inertia = np.zeros((3, 3))
        for mass, center, body_inertia in parts:
            d = center - com
            inertia += body_inertia + mass * (d @ d * np.eye(3) - np.outer(d, d))
        return total_mass, com, inertia


def solve_equilibrium(model, target_height, verbose=False):
    """Return equilibrium and default_joint_positions for base_link height in meters.

    Track the original stance to select among feasible configurations. Joint
    limits, torque limits, contact forces and target height are hard constraints.
    """
    target_height = float(target_height)
    if not np.isfinite(target_height) or target_height <= 0:
        raise ValueError('target_height must be finite and positive (meters)')
    q, tau, forces = ca.MX.sym('q', 12), ca.MX.sym('tau', 6), ca.MX.sym('forces', 6)
    pose = q[:6]
    y = ca.vertcat(q, tau, forces)
    reference = model.q_guess.copy()
    reference[2] = target_height
    gravity, J, contacts, _, _ = model.evaluate(q)
    balance = gravity - ca.vertcat(ca.MX.zeros(6), tau) - J.T @ forces
    # Anchor translation/yaw while allowing body pitch to find the balance point.
    equality = ca.vertcat(balance, contacts[2, :].T,
                          (contacts[:2, 0] + contacts[:2, 1]) / 2 - model.contact_midpoint_xy,
                          pose[5], pose[2] - target_height, q[8], q[11])
    friction = ca.vertcat(*[forces[3*i]**2 + forces[3*i+1]**2
                            - (model.friction[i] * forces[3*i+2])**2 for i in range(2)])
    objective = (0.5 * ca.dot(ca.DM([100, 100, 100, 1000, 1000, 1000]),
                              (pose - reference[:6])**2)
                 + 0.5 * 10 * ca.sumsqr(q[6:] - DEFAULT_JOINT_POSITIONS)
                 + 0.5e-3 * (ca.sumsqr(tau) + ca.sumsqr(forces)))
    constraints = ca.vertcat(equality, friction)
    solver = ca.nlpsol('equilibrium', 'ipopt', {'x': y, 'f': objective, 'g': constraints},
                       {'ipopt.print_level': 5 if verbose else 0, 'print_time': verbose,
                        'ipopt.sb': 'yes', 'ipopt.tol': 1e-10, 'ipopt.max_iter': 300,
                        'ipopt.constr_viol_tol': 1e-10, 'ipopt.bound_relax_factor': 0})
    joint_lower = np.array([model.joint_limits.get(name, [-np.inf, np.inf])[0] for name in JOINT_NAMES])
    joint_upper = np.array([model.joint_limits.get(name, [-np.inf, np.inf])[1] for name in JOINT_NAMES])
    lower = np.r_[[-np.inf, -np.inf, 0, -0.5, -1.2, -0.5], joint_lower, -model.torque_limits,
                  [-np.inf, -np.inf, 0, -np.inf, -np.inf, 0]]
    upper = np.r_[[np.inf, np.inf, np.inf, 0.5, 1.2, 0.5], joint_upper, model.torque_limits, np.full(6, np.inf)]
    f0 = np.array([0, 0, -model.mass * model.gravity[2] / 2] * 2)
    result = solver(x0=np.r_[reference, np.zeros(6), f0], lbx=lower, ubx=upper,
                    lbg=np.r_[np.zeros(equality.numel()), [-np.inf, -np.inf]],
                    ubg=np.zeros(constraints.numel()))
    if not solver.stats()['success']:
        raise RuntimeError(f"Could not find an equilibrium at target_height={target_height:g} m: "
                           f"{solver.stats()['return_status']}. The height may be unreachable within the limits.")
    value = vec(result['x'])
    q_value = value[:12]
    eq_function = ca.Function('equilibrium_residual', [y], [equality])
    residual = vec(eq_function(value))
    if np.linalg.norm(residual, np.inf) > 1e-7:
        raise RuntimeError(f'Equilibrium constraint error: {np.linalg.norm(residual, np.inf)}')
    objective_gradient = ca.Function('objective_gradient', [y], [ca.gradient(objective, y)])
    # KKT stationarity includes equality, inequality and variable-bound multipliers.
    jac = ca.Function('constraint_jacobian', [y], [ca.jacobian(constraints, y)])
    kkt = (vec(objective_gradient(value)) + np.asarray(jac(value)).T @ vec(result['lam_g'])
           + vec(result['lam_x']))
    return {'q': q_value, 'torques': value[12:18], 'forces': value[18:].reshape(2, 3),
            'target_height': target_height,
            'default_joint_positions': dict(zip(JOINT_NAMES, q_value[6:].tolist())),
            'residual': residual, 'kkt_inf': float(np.linalg.norm(kkt, np.inf)),
            'iterations': int(solver.stats()['iter_count'])}
