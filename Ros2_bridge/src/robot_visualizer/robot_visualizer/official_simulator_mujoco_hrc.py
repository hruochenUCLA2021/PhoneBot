"""
MuJoCo simulator wrapper using the *official* MuJoCo viewer (mujoco.viewer).

This is a drop-in alternative to `simulator_mujoco_hrc.py` which uses `mujoco_viewer`.
The official viewer supports drawing debug arrows by writing custom geoms into `viewer.user_scn`.
"""

from __future__ import annotations

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


class official_simulator_mujoco_v1:
    """
    Simple wrapper around MuJoCo model/data + official viewer handle.

    - load_model(): loads XML into self.model/self.data
    - create_viewer(): opens official viewer and stores it in self.viewer (mujoco.viewer.Handle)
    - clear_overlays(): resets viewer.user_scn.ngeom so we can redraw markers each frame
    - add_axis_arrow(): adds mjGEOM_ARROW into viewer.user_scn
    - render(): sync viewer with latest state + overlays
    """

    def __init__(self):
        self.model: mujoco.MjModel | None = None
        self.data: mujoco.MjData | None = None
        self.viewer: mujoco.viewer.Handle | None = None

    def load_model(self, path_xml: str):
        self.model = mujoco.MjModel.from_xml_path(path_xml)
        self.data = mujoco.MjData(self.model)

    def create_viewer(self):
        assert self.model is not None and self.data is not None
        # Official viewer handle supports __enter__/__exit__ and .close()
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    @property
    def is_alive(self) -> bool:
        v = self.viewer
        return bool(v is not None and v.is_running())

    def close(self):
        v = self.viewer
        self.viewer = None
        if v is not None:
            try:
                v.close()
            except Exception:
                pass

    def clear_overlays(self):
        v = self.viewer
        if v is None:
            return
        # Clear user overlay geoms each frame.
        v.user_scn.ngeom = 0

    def add_axis_arrow(
        self,
        start,
        direction,
        *,
        length: float = 0.1,
        color=(1.0, 0.0, 0.0, 1.0),
        radius: float = 0.02,
    ):
        """
        Add an RGB axis arrow marker to viewer.user_scn.

        Parameters:
        - start: xyz origin (world frame)
        - direction: xyz direction (world frame), will be normalized
        - length: arrow length (world units)
        - radius: arrow radius (world units)
        - color: rgba (float)
        """
        v = self.viewer
        if v is None:
            return
        scn = v.user_scn
        idx = int(scn.ngeom)
        if idx >= len(scn.geoms):
            return

        start = np.asarray(start, dtype=np.float64)
        direction = np.asarray(direction, dtype=np.float64)
        n = float(np.linalg.norm(direction))
        if not np.isfinite(n) or n < 1e-9:
            return
        direction = direction / n

        # Orient arrow so its +Z points along `direction`.
        z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        mat3 = rotation_matrix_from_vectors(z, direction).reshape((3, 3))

        # Match the convention used in `test_official_viewer.py` (known-good):
        # - Use pos=start directly.
        # - Use size[2]=length (not half-length).
        #
        # mjv_initGeom binding is strict about shapes/dtypes:
        #   size: float64 (3,1), pos: float64 (3,1), mat: float64 (9,1), rgba: float32 (4,1)
        size = np.array([float(radius), float(radius), float(length)], dtype=np.float64).reshape((3, 1))
        pos = start.reshape((3, 1)).astype(np.float64)
        mat = mat3.astype(np.float64).reshape((9, 1))
        rgba = np.array(color, dtype=np.float32).reshape((4, 1))

        mujoco.mjv_initGeom(
            scn.geoms[idx],
            int(mujoco.mjtGeom.mjGEOM_ARROW),
            size,
            pos,
            mat,
            rgba,
        )
        scn.ngeom = idx + 1

    def render(self):
        v = self.viewer
        if v is None:
            return
        v.sync()

