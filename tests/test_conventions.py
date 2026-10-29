"""Frame conventions: the stored calibration, the upside-down camera and transform I/O agree."""
from pathlib import Path

import numpy as np

from lidarzed.calibration.edges import to_camera
from lidarzed.camera import lens_intrinsics, split_stereo
from lidarzed.transforms import load_T, save_T

CAL = Path(__file__).resolve().parent.parent / "calibration" / "2026-10-28"


def project(T, K, p_lidar):
    pc = to_camera(np.atleast_2d(p_lidar), T)[0]
    return K[0, 0] * pc[0] / pc[2] + K[0, 2], K[1, 1] * pc[1] / pc[2] + K[1, 2], pc[2]


def test_upside_down_camera_geometry():
    """In the RAW image of the upside-down camera: forward lands near the center, up goes down, left goes right."""
    K, _ = lens_intrinsics(CAL / "SN17516.conf", "HD")
    for name in ("init_T_lidar_camera.json", "T_lidar_camera.json"):
        T = load_T(CAL / name)
        u0, v0, z0 = project(T, K, [2.0, 0, 0])
        assert z0 > 1.8                                  # in front of the camera
        assert abs(u0 - K[0, 2]) < 60 and abs(v0 - K[1, 2]) < 60
        _, v_up, _ = project(T, K, [2.0, 0, 0.5])
        u_left, _, _ = project(T, K, [2.0, 0.5, 0])
        assert v_up > v0                                 # world up -> image down (upside down)
        assert u_left > u0                               # world left -> image right (upside down)


def test_lens_file_is_a_real_calibration():
    KL, dL = lens_intrinsics(CAL / "SN17516.conf", "HD", "LEFT")
    KR, _ = lens_intrinsics(CAL / "SN17516.conf", "HD", "RIGHT")
    assert not np.allclose(KL, KR)          # placeholder files have identical left/right values
    assert dL[0] != 0


def test_split_stereo_upside_down_swaps_and_rotates():
    frame = np.zeros((2, 8, 3), np.uint8)
    frame[0, 0] = 1                          # top-left pixel of the camera's left image
    frame[0, 4] = 2                          # top-left pixel of the camera's right image
    left, right = split_stereo(frame, upside_down=True)
    assert left[1, 3, 0] == 2                # world-left view comes from the camera's right lens, rotated
    assert right[1, 3, 0] == 1


def test_transform_roundtrip(tmp_path):
    T = load_T(CAL / "T_lidar_camera.json")
    save_T(tmp_path / "t.json", T)
    assert np.allclose(load_T(tmp_path / "t.json"), T)
