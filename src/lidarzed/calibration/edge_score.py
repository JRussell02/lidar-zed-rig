"""Score camera-LiDAR calibrations: how far LiDAR depth edges land from image edges, in pixels.

For each session, stacks the hold-still scans, finds depth discontinuities (object outlines),
projects them with each calibration and measures the distance to the nearest image edge. Lower =
better aligned. Reported per distance band: a translation error shows mostly on NEAR objects, a
rotation error everywhere.

  lidarzed-edge-score sessions/calib0[1-5] --T calibration/2026-10-28/init_T_lidar_camera.json \
      calibration/2026-10-28/T_lidar_camera.json
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
from .edges import depth_edge_mask, image_edge_distance, project, to_camera


def session_data(session, n_scans=10):
    """(edge points in LiDAR frame, K, dist, image edge-distance map) from the hold-still window."""
    K, dist = lens_from_session(session)
    ims = [(int(r["host_ns"]), r["file"]) for r in csv.DictReader(open(session / "zed" / "frames.csv"))]
    img_ns = np.array([t for t, _ in ims])
    src, info = open_osf(session / "lidar.osf")
    lut = core.XYZLut(info)
    pts, first = [], None
    for fr in frames(src):
        start = frame_start_ns(fr)
        if start < img_ns[0]:
            continue
        first = first or start
        rng = fr.field(core.ChanField.RANGE)
        pts.append(lut(rng)[depth_edge_mask(info, rng)])
        if len(pts) >= n_scans:
            break
    src.close()
    i = int(np.argmin(np.abs(img_ns - first)))
    gray = cv2.imread(str(session / "zed" / "left" / ims[i][1]), cv2.IMREAD_GRAYSCALE)
    return np.vstack(pts), K, dist, image_edge_distance(gray)


def score(P, K, dist, dt, T):
    uv, z = project(to_camera(P, T), K, dist, dt.shape)
    e = dt[uv[:, 1].astype(int), uv[:, 0].astype(int)]
    out = {"all": np.median(e)}
    for name, lo, hi in (("<2m", 0, 2), ("2-4m", 2, 4), (">4m", 4, 99)):
        sel = (z >= lo) & (z < hi)
        out[name] = np.median(e[sel]) if sel.sum() > 200 else float("nan")
    return out, len(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--T", nargs="+", required=True, help="calibration JSON files to compare")
    args = ap.parse_args()
    Ts = {Path(p).stem: load_T(p) for p in args.T}
    totals = {k: [] for k in Ts}
    print(f"{'session':10s} {'calibration':28s} {'median px':>9s} {'<2m':>6s} {'2-4m':>6s} {'>4m':>6s}  edge pts")
    for s in args.sessions:
        P, K, dist, dt = session_data(s)
        for name, T in Ts.items():
            r, n = score(P, K, dist, dt, T)
            totals[name].append(r["all"])
            print(f"{s.name:10s} {name:28s} {r['all']:9.2f} {r['<2m']:6.2f} {r['2-4m']:6.2f} {r['>4m']:6.2f}  {n:,}")
        print()
    print("mean of medians: " + "  ".join(f"{k}={np.mean(v):.2f}px" for k, v in totals.items()))


if __name__ == "__main__":
    main()
