"""Camera-LiDAR transforms stored as JSON.

Format: {"translation_m": [x, y, z], "quaternion_xyzw": [qx, qy, qz, qw]} = the camera's pose in the
Ouster SENSOR frame (+X away from the connector, origin at the base center). The calibration tool's
calib.json (results.T_lidar_camera = [tx, ty, tz, qx, qy, qz, qw]) is accepted too.
"""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def load_T(path, key="T_lidar_camera"):
    """4x4 camera pose in the LiDAR sensor frame (maps camera points to LiDAR points)."""
    d = json.loads(Path(path).read_text())
    if "translation_m" in d:
        t, q = d["translation_m"], d["quaternion_xyzw"]
    else:
        v = d["results"][key]
        t, q = v[:3], v[3:]
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rotation.from_quat(q).as_matrix(), t
    return T


def save_T(path, T, note=""):
    Path(path).write_text(json.dumps({
        "frame_note": note,
        "translation_m": [float(v) for v in T[:3, 3]],
        "quaternion_xyzw": [float(v) for v in Rotation.from_matrix(T[:3, :3]).as_quat()],
    }, indent=2))
