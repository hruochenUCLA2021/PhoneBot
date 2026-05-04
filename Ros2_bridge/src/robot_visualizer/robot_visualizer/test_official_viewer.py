#!/usr/bin/env python3

"""
Minimal MuJoCo *official* viewer marker test.

This is an alternative to `test_mujoco_viewer_arrow.py` that uses MuJoCo's official viewer:
`mujoco.viewer.launch_passive`.

It attempts to draw 3 RGB axis arrows at the origin using the viewer "user scene" overlay:
- X axis: red
- Y axis: green
- Z axis: blue
"""

import time

import numpy as np
import mujoco
import mujoco.viewer


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


def add_axis_arrow_official(viewer, origin, direction, length, rgba):
    """
    Add a mjGEOM_ARROW to the official viewer's user scene.

    The official viewer exposes an overlay scene at `viewer.user_scn` (mjvScene),
    which has a fixed-size `geoms` array and an `ngeom` counter.
    """
    scene = viewer.user_scn
    idx = int(scene.ngeom)
    if idx >= len(scene.geoms):
        return

    origin = np.asarray(origin, dtype=np.float64)
    direction = np.asarray(direction, dtype=np.float64)
    if np.linalg.norm(direction) < 1e-9:
        return
    direction = direction / np.linalg.norm(direction)

    # Build orientation matrix that maps +Z to direction.
    z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    mat = rotation_matrix_from_vectors(z, direction).reshape(3, 3)

    # NOTE: mujoco.mjv_initGeom binding is strict about shapes/dtypes:
    #   size: float64 (3,1)
    #   pos : float64 (3,1)
    #   mat : float64 (9,1)
    #   rgba: float32 (4,1)
    size = np.array([0.02, 0.02, float(length)], dtype=np.float64).reshape((3, 1))
    pos = origin.astype(np.float64).reshape((3, 1))
    mat = mat.astype(np.float64).reshape((9, 1))
    rgba = np.array(rgba, dtype=np.float32).reshape((4, 1))

    # mjv_initGeom signature in MuJoCo C API:
    # mjv_initGeom(geom, type, size, pos, mat, rgba)
    mujoco.mjv_initGeom(
        scene.geoms[idx],
        int(mujoco.mjtGeom.mjGEOM_ARROW),
        size,
        pos,
        mat,
        rgba,
    )
    scene.ngeom = idx + 1


def main():
    print("mujoco:", mujoco.__version__)
    print("official viewer available:", hasattr(mujoco, "viewer"))

    xml = r"""
<mujoco model="official_viewer_arrow_test">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="floor" type="plane" size="2 2 0.1" rgba="0.2 0.2 0.2 1"/>
  </worldbody>
</mujoco>
""".strip()

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    origin = np.array([0.0, 0.0, 0.05], dtype=np.float64)
    length = 0.25

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t0 = time.time()
        while viewer.is_running() and (time.time() - t0) < 10.0:
            mujoco.mj_forward(model, data)

            # Clear overlay geoms and add 3 arrows.
            viewer.user_scn.ngeom = 0
            add_axis_arrow_official(viewer, origin, [1, 0, 0], length=length, rgba=(1, 0, 0, 0.9))
            add_axis_arrow_official(viewer, origin, [0, 1, 0], length=length, rgba=(0, 1, 0, 0.9))
            add_axis_arrow_official(viewer, origin, [0, 0, 1], length=length, rgba=(0, 0, 1, 0.9))

            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()

