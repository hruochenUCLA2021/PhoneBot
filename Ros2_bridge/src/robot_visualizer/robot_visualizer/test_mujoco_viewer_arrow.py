#!/usr/bin/env python3

"""
Minimal mujoco_viewer marker test.

Goal: verify whether `mujoco_viewer` can draw arrow markers in your current environment.
It opens a viewer on a tiny MuJoCo model and adds 3 arrows at the origin:
- X axis: red
- Y axis: green
- Z axis: blue

If your mujoco_viewer + mujoco versions are incompatible (e.g. MjvGeom has no attribute 'texid'),
this script should reproduce the crash quickly.
"""

import time

import numpy as np
import mujoco
import mujoco_viewer


def rotation_matrix_from_vectors(vector1, vector2):
    """Rodrigues rotation matrix that rotates vector1 -> vector2."""
    v1 = np.asarray(vector1, dtype=np.float64)
    v2 = np.asarray(vector2, dtype=np.float64)
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    v = np.cross(v1, v2)
    c = float(np.dot(v1, v2))
    if np.allclose(v, [0.0, 0.0, 0.0]):
        return np.eye(3)
    V = np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=np.float64,
    )
    R = np.eye(3) + V + (V @ V) * ((1.0 - c) / (np.linalg.norm(v) ** 2))
    return R


def add_axis_arrow(viewer, origin, direction, length, rgba):
    """
    Add a mjGEOM_ARROW marker.

    Many marker APIs expect a rotation matrix in 'mat' rather than a direction vector.
    We build an orientation that maps +Z to the desired arrow direction.
    """
    origin = np.asarray(origin, dtype=np.float64)
    direction = np.asarray(direction, dtype=np.float64)
    if np.linalg.norm(direction) < 1e-9:
        return
    direction = direction / np.linalg.norm(direction)
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    mat = rotation_matrix_from_vectors(z, direction)
    viewer.add_marker(
        pos=origin,
        type=mujoco.mjtGeom.mjGEOM_ARROW,
        size=[0.02, 0.02, float(length)],
        rgba=rgba,
        mat=mat,
    )


def main():
    print("mujoco:", mujoco.__version__)
    print("mujoco_viewer:", getattr(mujoco_viewer, "__version__", "unknown"))
    print("mujoco_viewer file:", getattr(mujoco_viewer, "__file__", "unknown"))

    # Minimal model: just a ground plane so we get a camera/view.
    xml = r"""
<mujoco model="viewer_arrow_test">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.2 0.2 0.2 1"/>
  </worldbody>
</mujoco>
""".strip()

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    viewer = mujoco_viewer.MujocoViewer(model, data)

    origin = np.array([0.0, 0.0, 0.05], dtype=np.float64)
    length = 0.25

    t0 = time.time()
    try:
        while viewer.is_alive:
            mujoco.mj_forward(model, data)

            # Clear old markers if supported.
            for attr in ("markers", "_markers"):
                m = getattr(viewer, attr, None)
                if isinstance(m, list):
                    m.clear()

            # X (red), Y (green), Z (blue)
            add_axis_arrow(viewer, origin, [1, 0, 0], length=length, rgba=(1, 0, 0, 0.9))
            add_axis_arrow(viewer, origin, [0, 1, 0], length=length, rgba=(0, 1, 0, 0.9))
            add_axis_arrow(viewer, origin, [0, 0, 1], length=length, rgba=(0, 0, 1, 0.9))

            viewer.render()

            # Exit after a short time so it doesn't run forever in terminal.
            if time.time() - t0 > 10.0:
                break
    finally:
        try:
            viewer.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

