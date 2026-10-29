"""Draw OS1 points (colored by distance) over the ZED image to check a calibration by eye.

Uses the hold-still scans (rig not moving, so they stack into a denser cloud) and the camera frame
nearest the first of them. If the calibration is right, depth edges land on the image edges.

  lidarzed-overlay sessions/calib05 --T calibration/2026-10-28/T_lidar_camera.json --out overlay.jpg --upright
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from ouster.sdk import core

from ..camera import lens_from_session
from ..lidar import frame_start_ns, frames, open_osf
from ..transforms import load_T
from .edges import to_camera


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--T", required=True, help="calibration JSON")
    ap.add_argument("--out", required=True)
    ap.add_argument("--scans", type=int, default=10, help="hold-still scans to stack [10]")
    ap.add_argument("--max-range", type=float, default=12.0)
    ap.add_argument("--upright", action="store_true", help="rotate the output 180° (camera mounted upside down)")
    args = ap.parse_args()

    K, dist = lens_from_session(args.session)
    ims = [(int(r["host_ns"]), r["file"]) for r in csv.DictReader(open(args.session / "zed" / "frames.csv"))]
    img_ns = np.array([t for t, _ in ims])

    src, info = open_osf(args.session / "lidar.osf")
    lut = core.XYZLut(info)
    pts, first_ns = [], None
    for fr in frames(src):
        start = frame_start_ns(fr)
        if start < img_ns[0]:
            continue
        first_ns = first_ns or start
        rng = fr.field(core.ChanField.RANGE)
        pts.append(lut(rng)[rng > 0])
        if len(pts) >= args.scans:
            break
    src.close()
    if not pts:
        raise SystemExit("no complete scans in the hold window")

    i = int(np.argmin(np.abs(img_ns - first_ns)))
    img = cv2.imread(str(args.session / "zed" / "left" / ims[i][1]))
    pc = to_camera(np.vstack(pts), load_T(args.T))
    pc = pc[pc[:, 2] > 0.3]
    d_all = np.linalg.norm(pc, axis=1)
    uv_all = cv2.projectPoints(pc, np.zeros(3), np.zeros(3), K, dist)[0].reshape(-1, 2)
    h, w = img.shape[:2]
    m = (uv_all[:, 0] >= 0) & (uv_all[:, 0] < w) & (uv_all[:, 1] >= 0) & (uv_all[:, 1] < h) & (d_all < args.max_range)
    uv, d = uv_all[m].astype(int), d_all[m]

    order = np.argsort(-d)  # far first so near points stay on top
    colors = cv2.applyColorMap(np.clip(255 * (d[order] - 0.5) / 6.0, 0, 255).astype(np.uint8)[:, None],
                               cv2.COLORMAP_TURBO)[:, 0]
    out = (img * 0.6).astype(np.uint8)  # dim the photo so the colored points stand out
    for (u, v), c in zip(uv[order][::2], colors[::2]):
        cv2.circle(out, (int(u), int(v)), 1, tuple(int(x) for x in c), -1)
    if args.upright:
        out = cv2.rotate(out, cv2.ROTATE_180)
    cv2.putText(out, f"{args.session.name}  {Path(args.T).name}  {m.sum():,} pts", (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imwrite(args.out, out)
    print(f"{args.out}: {m.sum():,} points in view from {len(pts)} scans, image {ims[i][1]} "
          f"({(img_ns[i] - first_ns) / 1e6:+.0f} ms)")


if __name__ == "__main__":
    main()
