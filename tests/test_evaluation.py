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
