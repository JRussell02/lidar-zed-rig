"""ICP alignment in lidarzed-eval recovers a known offset between two synthetic room scans."""
import numpy as np
from scipy.spatial import cKDTree

from lidarzed import evaluation as ev


def synthetic_room(rng, n=40_000):
    """Surfaces of an 8 x 6 x 3 m room with a table and a box."""
    def plane(o, u, v, k):
        a = rng.random((k, 2))
        return np.asarray(o) + a[:, :1] * np.asarray(u) + a[:, 1:] * np.asarray(v)
    parts = [plane([0, 0, 0], [8, 0, 0], [0, 6, 0], n), plane([0, 0, 3], [8, 0, 0], [0, 6, 0], n),
             plane([0, 0, 0], [8, 0, 0], [0, 0, 3], n // 2), plane([0, 6, 0], [8, 0, 0], [0, 0, 3], n // 2),
             plane([0, 0, 0], [0, 6, 0], [0, 0, 3], n // 2), plane([8, 0, 0], [0, 6, 0], [0, 0, 3], n // 2),
             plane([2, 2, 0.75], [2, 0, 0], [0, 1, 0], n // 8),
             plane([6, 1, 0], [0.5, 0, 0], [0, 0, 1], n // 16), plane([6, 1, 0], [0, 0.5, 0], [0, 0, 1], n // 16)]
    return np.vstack(parts)


def test_icp_recovers_known_offset():
    rng = np.random.default_rng(0)
    ref = synthetic_room(rng) + rng.normal(0, 0.005, (1, 3))
    T_true = ev.rot_z(3.0)
    T_true[:3, 3] = [0.15, -0.08, 0.02]
    moved = ev.transform(synthetic_room(rng), T_true)

    ref_d = ev.voxel_down(ref, 0.05)
    src_d = ev.voxel_down(moved, 0.05)
    T = ev.icp(src_d, cKDTree(ref_d), ref_d, np.eye(4))

    T_est_inv = T @ T_true            # should be identity if T undoes T_true
    assert np.linalg.norm(T_est_inv[:3, 3]) < 0.01
    yaw = np.degrees(np.arctan2(T_est_inv[1, 0], T_est_inv[0, 0]))
    assert abs(yaw) < 0.2


def test_ply_roundtrip(tmp_path):
    pts = np.random.default_rng(1).random((100, 3))
    f = tmp_path / "x.ply"
    ev.write_ply(f, pts)
    assert np.allclose(ev.read_ply(f), pts, atol=1e-6)


def test_colored_ply_roundtrip_and_voxel_colors(tmp_path):
    rng = np.random.default_rng(2)
    pts, rgb = rng.random((100, 3)), rng.integers(0, 256, (100, 3)).astype(float)
    f = tmp_path / "c.ply"
    ev.write_ply(f, pts, rgb)
    p2, c2 = ev.read_ply_rgb(f)
    assert np.allclose(p2, pts, atol=1e-6) and np.array_equal(c2, rgb)
    _, c = ev.voxel_down(np.zeros((2, 3)), 0.02, np.array([[0, 0, 0], [200, 100, 50]], float))
    assert np.allclose(c, [[100, 50, 25]])


def test_global_align_recovers_any_start_pose():
    rng = np.random.default_rng(3)
    ref = synthetic_room(rng)
    tilt = np.eye(4)
    tilt[:3, :3] = ev.rot_between(np.array([0.0, 0.0, 1.0]), np.array([np.sin(np.radians(8)), 0, np.cos(np.radians(8))]))
    T_true = ev.rot_z(130.0) @ tilt
    T_true[:3, 3] = [2.5, -1.0, 0.3]
    moved = ev.transform(synthetic_room(rng), T_true)

    T = ev.global_align(moved, ref)
    src_d, ref_d = ev.voxel_down(moved, 0.05), ev.voxel_down(ref, 0.05)
    T = ev.icp(src_d, cKDTree(ref_d), ref_d, T)

    err = T @ T_true                  # identity if T undoes T_true
    assert np.linalg.norm(err[:3, 3]) < 0.02
    assert np.degrees(np.arccos(np.clip((np.trace(err[:3, :3]) - 1) / 2, -1, 1))) < 0.5


def test_path_report_flags_height_and_tilt():
    poses = np.repeat(np.eye(4)[None], 4, axis=0)
    poses[:, :3, 3] = [[0, 0, 0], [1, 0, 0], [1, 0, -2], [0, 0, 0]]
    poses[2, :3, :3] = ev.rot_between(np.array([0.0, 0.0, 1.0]), np.array([0.0, np.sin(0.5), np.cos(0.5)]))
    r = ev.path_report(poses)
    assert np.isclose(r["length"], 1 + 2 + np.sqrt(5))
    assert (r["z_min"], r["z_max"], r["max_jump"]) == (-2, 0, np.sqrt(5))
    assert np.isclose(r["tilt_max"], np.degrees(0.5)) and r["end_gap"] == 0


def test_merge_colors_only_from_colored_points():
    pts = np.array([[0.001, 0, 0], [0.002, 0, 0], [0.5, 0, 0]])
    rgb = np.array([[200, 0, 0], [128, 128, 128], [128, 128, 128]], float)
    out, col, colored = ev.voxel_down(pts, 0.02, rgb, np.array([True, False, False]))
    first = np.argmin(out[:, 0])
    assert colored[first] and not colored[1 - first]
    assert np.allclose(col[first], [200, 0, 0])                         # the gray point doesn't dilute it


def tilt_about_x(deg):
    T = np.eye(4)
    T[:3, :3] = ev.rot_between(np.array([0.0, 0.0, 1.0]), np.array([0, np.sin(np.radians(deg)), np.cos(np.radians(deg))]))
    return T


def test_up_direction_finds_true_vertical_in_tilted_room():
    rng = np.random.default_rng(4)
    T = tilt_about_x(25.0)                          # a scan whose start pose was tilted 25°
    room = ev.voxel_down(ev.transform(synthetic_room(rng), T), 0.1)
    up = ev.up_direction(ev.normals(room), max_deg=50)
    assert np.degrees(np.arccos(up @ T[:3, 2])) < 0.5


def test_path_report_uses_true_vertical_for_tilted_start():
    T = tilt_about_x(25.0)
    poses = np.repeat(T[None], 50, axis=0)          # rig held tilted, walking flat for 5 m
    poses[:, :3, 3] = np.c_[np.zeros(50), np.linspace(0, 5, 50), np.zeros(50)]
    start_frame = np.linalg.inv(T) @ poses          # SLAM's world frame is the tilted start pose
    naive = ev.path_report(start_frame)
    fixed = ev.path_report(start_frame, up=np.linalg.inv(T)[:3, 2])
    assert naive["z_max"] - naive["z_min"] > 2.0    # the old false alarm
    assert fixed["z_max"] - fixed["z_min"] < 1e-6
    assert np.isclose(fixed["tilt_start"], 25.0) and np.isclose(fixed["tilt_max"], 25.0)
