"""Color a SLAM'd OS1 recording with the ZED images -> colored point cloud (PLY for CloudCompare).

For every LiDAR scan:
  1. points go to the world frame with SLAM's per-column poses (motion during the spin is corrected),
  2. the camera image captured closest to when the beam swept the camera's view is chosen
     (capture time = frames.csv arrival time - camera delay),
  3. the camera's world pose at that capture time = SLAM pose at that instant x camera-LiDAR calibration,
  4. points are projected into the image; a coarse depth test drops points hidden from the camera
     (the camera sits ~8 cm from the LiDAR, so it can't see everything the LiDAR sees),
  5. colors are averaged per voxel across all scans.

Steps:
  ouster-cli source sessions/X/lidar.osf slam -v 0.1 save processed/X_slam.osf
  lidarzed-colorize sessions/X processed/X_slam.osf --out processed/X_color.ply

--T and --delay default to calibration/2026-10-28/ (relative to the current folder).
"""
import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ouster.sdk import core

from .camera import lens_from_session
from .lidar import column_host_ns, frames, open_osf
from .transforms import load_T

CALIBRATION = Path("calibration/2026-10-28")
HALF_FOV_DEG = 55          # select LiDAR points roughly in front of the camera before projecting
ZBUF_BLOCK = 4             # px; depth test resolution
ZBUF_TOL = 0.03            # keep points within 3% (+2 cm) of the nearest depth in their block


class VoxelAccumulator:
    """Running per-voxel sums of position and color, merged in chunks to bound memory."""

    def __init__(self, size):
        self.size = size
        self.keys = np.empty(0, np.int64)
        self.xyz = np.empty((0, 3))
        self.rgb = np.empty((0, 3))
        self.n = np.empty(0)
        self.pending = []

    def _key(self, p):
        q = np.floor(p / self.size).astype(np.int64) + (1 << 20)
        return (q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2]

    def add(self, xyz, rgb):
        self.pending.append((xyz, rgb))
        if sum(len(x) for x, _ in self.pending) > 4_000_000:
            self.flush()

    def flush(self):
        if not self.pending:
            return
        xyz = np.vstack([x for x, _ in self.pending]).astype(np.float64)
        rgb = np.vstack([c for _, c in self.pending]).astype(np.float64)
        self.pending = []
        keys = np.concatenate([self.keys, self._key(xyz)])
        sx = np.vstack([self.xyz, xyz]); sc = np.vstack([self.rgb, rgb])
        n = np.concatenate([self.n, np.ones(len(xyz))])
        uk, inv = np.unique(keys, return_inverse=True)
        inv = inv.ravel()
        self.keys = uk
        self.xyz = np.zeros((len(uk), 3)); np.add.at(self.xyz, inv, sx)
        self.rgb = np.zeros((len(uk), 3)); np.add.at(self.rgb, inv, sc)
        self.n = np.zeros(len(uk)); np.add.at(self.n, inv, n)

    def result(self):
        self.flush()
        return self.xyz / self.n[:, None], self.rgb / self.n[:, None], self.n


def write_ply(path, xyz, rgb):
    rec = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                    ("red", "u1"), ("green", "u1"), ("blue", "u1")])
    rec["x"], rec["y"], rec["z"] = xyz.T
    rec["red"], rec["green"], rec["blue"] = np.clip(np.round(rgb), 0, 255).astype(np.uint8).T
    with open(path, "wb") as f:
        f.write((f"ply\nformat binary_little_endian 1.0\nelement vertex {len(rec)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n").encode())
        f.write(rec.tobytes())



def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path, help="lidarzed-record session (zed/left images + frames.csv)")
    ap.add_argument("slam_osf", type=Path, help="OSF written by `ouster-cli ... slam ... save`")
    ap.add_argument("--out", required=True)
    ap.add_argument("--T", default=CALIBRATION / "T_lidar_camera.json", help="camera-LiDAR calibration JSON")
    ap.add_argument("--delay", default=CALIBRATION / "camera_delay.json", help="JSON with camera_delay_ms, or a number in ms")
    ap.add_argument("--voxel", type=float, default=0.02, help="output point spacing in m [0.02]")
    ap.add_argument("--min-range", type=float, default=0.6)
    ap.add_argument("--max-range", type=float, default=15.0)
    ap.add_argument("--every", type=int, default=1, help="use every Nth scan [1]")
    args = ap.parse_args()

    delay_ms = float(args.delay) if str(args.delay).replace(".", "", 1).lstrip("-").isdigit() \
        else json.loads(Path(args.delay).read_text())["camera_delay_ms"]
    T_lc = load_T(args.T)
    K, dist = lens_from_session(args.session)
    images = [(int(r["host_ns"]), r["file"]) for r in csv.DictReader(open(args.session / "zed" / "frames.csv"))]
    cap_ns = np.array([t for t, _ in images]) - int(delay_ms * 1e6)

    src, info = open_osf(args.slam_osf)
    lut = core.XYZLut(info)
    acc = VoxelAccumulator(args.voxel)
    used = skipped = 0
    t_start = time.monotonic()
    img_cache = {}
    for k, fr in enumerate(frames(src)):
        if k % args.every:
            continue
        col_ns = column_host_ns(fr)
        poses = np.asarray(fr.body_to_world)                       # (W, 4, 4), per measurement column
        rng = fr.field(core.ChanField.RANGE)
        xyz = lut(rng)                                               # (H, W, 3) sensor frame
        r = np.linalg.norm(xyz, axis=2)
        az = np.degrees(np.arctan2(xyz[..., 1], xyz[..., 0]))
        sel = (rng > 0) & (r > args.min_range) & (r < args.max_range) & (np.abs(az) < HALF_FOV_DEG) & (xyz[..., 0] > 0)
        rows, cols = np.nonzero(sel)
        if len(cols) < 1000:
            skipped += 1
            continue
        # image captured closest to when the beam swept the camera's view
        t_view = int(np.median(col_ns[cols]))
        i = int(np.argmin(np.abs(cap_ns - t_view)))
        if abs(cap_ns[i] - t_view) > 60e6:
            skipped += 1
            continue
        # points -> world with their own column poses
        p = xyz[rows, cols]
        R = poses[cols, :3, :3]; t = poses[cols, :3, 3]
        pw = np.einsum("nij,nj->ni", R, p) + t
        # camera pose at its capture time: SLAM pose of the column measured closest to that instant
        cam_col = int(np.argmin(np.abs(col_ns - cap_ns[i])))
        T_wc = poses[cam_col] @ T_lc
        T_cw = np.linalg.inv(T_wc)
        pc = pw @ T_cw[:3, :3].T + T_cw[:3, 3]
        front = pc[:, 2] > 0.3
        pw, pc = pw[front], pc[front]
        uv = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
        if i not in img_cache:
            img_cache = {i: cv2.imread(str(args.session / "zed" / "left" / images[i][1]))}
        img = img_cache[i]
        h, w = img.shape[:2]
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w - 1) & (uv[:, 1] >= 0) & (uv[:, 1] < h - 1)
        pw, pc, uv = pw[inside], pc[inside], uv[inside]
        # coarse depth test: keep points near the closest depth in their pixel block
        d = pc[:, 2]
        block = (uv[:, 1] // ZBUF_BLOCK).astype(np.int64) * (w // ZBUF_BLOCK + 1) + (uv[:, 0] // ZBUF_BLOCK).astype(np.int64)
        dmin = np.full(block.max() + 1, np.inf)
        np.minimum.at(dmin, block, d)
        vis = d <= dmin[block] * (1 + ZBUF_TOL) + 0.02
        pw, uv = pw[vis], uv[vis]
        bgr = img[np.round(uv[:, 1]).astype(int), np.round(uv[:, 0]).astype(int)]
        acc.add(pw, bgr[:, ::-1])
        used += 1
        if used % 100 == 0:
            print(f"  {used} scans colored ({time.monotonic() - t_start:.0f} s)")
    src.close()

    xyz, rgb, n = acc.result()
    write_ply(args.out, xyz, rgb)
    print(f"{args.out}: {len(xyz):,} colored points ({args.voxel * 100:.0f} cm voxels) from {used} scans, "
          f"{skipped} skipped; camera delay {delay_ms:.0f} ms; {time.monotonic() - t_start:.0f} s")


if __name__ == "__main__":
    main()
