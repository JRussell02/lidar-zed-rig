"""Voxel color averaging and PLY output used by lidarzed-colorize."""
import numpy as np

from lidarzed.coloring import VoxelAccumulator, write_ply
from lidarzed.render import read_ply


def test_voxel_accumulator_averages_across_chunks():
    acc = VoxelAccumulator(0.1)
    acc.add(np.array([[0.01, 0.01, 0.01]]), np.array([[100, 0, 0]]))
    acc.flush()
    acc.add(np.array([[0.03, 0.02, 0.01], [1.0, 1.0, 1.0]]), np.array([[200, 0, 0], [0, 50, 0]]))
    xyz, rgb, n = acc.result()
    assert len(xyz) == 2
    first = np.argmin(np.linalg.norm(xyz, axis=1))
    assert n[first] == 2 and rgb[first][0] == 150     # same voxel: colors averaged
    assert np.allclose(xyz[first], [0.02, 0.015, 0.01])


def test_colored_ply_roundtrip(tmp_path):
    xyz = np.array([[0, 0, 0], [1, 2, 3]], float)
    rgb = np.array([[255, 0, 10], [0, 128, 255]], float)
    f = tmp_path / "c.ply"
    write_ply(f, xyz, rgb)
    p, bgr = read_ply(f)
    assert np.allclose(p, xyz)
    assert (bgr[:, ::-1] == rgb).all()
