"""MeshCat visualization for pineapple_v0. Given code, no TODOs.

Renders the MJCF visual meshes at one or more configurations, with markers for the
wheel contacts and the center of mass. Shared by lecture 3 (showing a solved standing
pose) and lecture 4 (comparing the reference stance to the equilibrium).

In a notebook, finish the cell with `viewer.jupyter_cell()` to embed the 3-D view.
MeshCat is an optional dependency; nothing else in the course needs it.
"""

import numpy as np

from pineapple_model import quaternion_rotation, vec

DEFAULT_CAMERA = (1.1, 0.9, 1.6)
POSE_SPACING = 0.35          # lateral gap between poses shown side by side
CONTACT_COLOR = 0x2A9D8F
COM_COLOR = 0xE76F51
GROUND_COLOR = 0xC9CCD1


def create_meshcat_viewer(camera=DEFAULT_CAMERA):
    """Start a local MeshCat server and aim the camera at the scene."""
    try:
        import meshcat
    except ImportError as exc:
        raise RuntimeError(
            'MeshCat is required for the 3-D view. Install it with\n'
            '    python3 -m pip install "meshcat>=0.3.2"\n'
            'Everything else in this homework works without it.') from exc
    viewer = meshcat.Visualizer()
    # MeshCat's camera child uses y-up coordinates; this looks across the poses
    # rather than along the axis that separates them.
    viewer['/Cameras/default/rotated/<object>'].set_property('position', list(camera))
    return viewer


def close_meshcat_viewer(viewer):
    """Release the locally owned server (MeshCat 0.3.2's close() is broken)."""
    import subprocess
    viewer.window.zmq_socket.close(linger=0)
    process = viewer.window.server_proc
    if process is not None and process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def draw_ground(viewer, size=1.6):
    """A translucent slab at z = 0 so the wheels have something to stand on."""
    from meshcat import geometry

    viewer['ground'].set_object(
        geometry.Box([size, size, 0.002]),
        geometry.MeshLambertMaterial(color=GROUND_COLOR, opacity=0.65, transparent=True))
    placement = np.eye(4)
    placement[2, 3] = -0.001
    viewer['ground'].set_transform(placement)


def draw_pose(viewer, model, q, name, offset=0.0, mesh_cache=None):
    """Draw one configuration: the MJCF visual meshes plus contact and CoM markers."""
    from meshcat import geometry

    mesh_cache = {} if mesh_cache is None else mesh_cache
    scene = viewer['pineapple'][name]
    placement = np.eye(4)
    placement[1, 3] = offset
    scene.set_transform(placement)

    transforms = np.asarray(model.transforms(q))
    for index, body in enumerate(model.body_names):
        scene[body].set_transform(transforms[:, 4 * index:4 * index + 4])
    for index, visual in enumerate(model.visuals):
        path = visual['mesh_path']
        if path not in mesh_cache:
            mesh_cache[path] = geometry.StlMeshGeometry.from_file(str(path))
        rgba = visual['rgba']
        rgb = np.round(255 * np.asarray(rgba[:3])).astype(int)
        node = scene[visual['body']][f'visual_{index}']
        node.set_object(mesh_cache[path], geometry.MeshLambertMaterial(
            color=int((rgb[0] << 16) | (rgb[1] << 8) | rgb[2]),
            opacity=float(rgba[3]), transparent=bool(rgba[3] < 1)))
        node.set_transform(visual['transform'])

    # Markers hang off the pose group, not off a body, so they take world coordinates.
    _, _, contacts, com, _ = model.evaluate(q)
    contacts, com = np.asarray(contacts), vec(com)
    for side in range(contacts.shape[1]):
        marker = np.eye(4)
        marker[:3, 3] = contacts[:, side]
        node = scene[f'contact_{side}']
        node.set_object(geometry.Sphere(0.012),
                        geometry.MeshLambertMaterial(color=CONTACT_COLOR))
        node.set_transform(marker)
    marker = np.eye(4)
    marker[:3, 3] = com
    scene['com'].set_object(geometry.Sphere(0.018),
                            geometry.MeshLambertMaterial(color=COM_COLOR))
    scene['com'].set_transform(marker)
    # Vertical drop line from the center of mass to the ground.
    drop = np.eye(4)
    drop[:3, 3] = [com[0], com[1], com[2] / 2]
    scene['com_drop'].set_object(
        geometry.Cylinder(max(com[2], 1e-6), 0.002),
        geometry.MeshLambertMaterial(color=COM_COLOR, opacity=0.5, transparent=True))
    # MeshCat cylinders run along their local y axis; stand this one upright.
    drop[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    scene['com_drop'].set_transform(drop)
    return len(model.visuals) + contacts.shape[1] + 2


def show_poses(viewer, model, poses, spacing=POSE_SPACING):
    """Draw labelled configurations side by side, centered on the origin.

    `poses` is a sequence of (q, label) pairs. Returns the number of objects drawn.
    """
    poses = list(poses)
    if not poses:
        raise ValueError('show_poses needs at least one (q, label) pair')
    draw_ground(viewer, size=max(1.6, spacing * (len(poses) + 1)))
    mesh_cache, objects = {}, 1
    start = -spacing * (len(poses) - 1) / 2
    for index, (q, label) in enumerate(poses):
        objects += draw_pose(viewer, model, q, label, start + index * spacing, mesh_cache)
    return objects
