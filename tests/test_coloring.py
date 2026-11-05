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


def test_voxel_accumulator_keeps_uncolored_geometry():
    acc = VoxelAccumulator(0.1)
    acc.add(np.array([[0.01, 0.01, 0.01]]), np.array([[90, 0, 0]]))
    acc.add(np.array([[0.05, 0.01, 0.01], [1.0, 1.0, 1.0]]))              # geometry only
    xyz, rgb, nc = acc.result()
    first = np.argmin(np.linalg.norm(xyz, axis=1))
    assert np.allclose(xyz[first], [0.03, 0.01, 0.01])                  # position uses all points
    assert nc[first] == 1 and rgb[first][0] == 90                       # color only from the colored one
    assert nc[1 - first] == 0


def test_ply_with_colored_flag_roundtrip(tmp_path):
    from lidarzed.evaluation import read_ply_colored
    xyz = np.array([[0, 0, 0], [1, 2, 3]], float)
    f = tmp_path / "a.ply"
    write_ply(f, xyz, np.array([[255, 0, 10], [128, 128, 128]], float), np.array([True, False]))
    p, rgb, colored = read_ply_colored(f)
    assert np.allclose(p, xyz) and list(colored) == [True, False] and rgb[0, 0] == 255
    assert read_ply(f)[1].shape == (2, 3)                               # render still reads it
