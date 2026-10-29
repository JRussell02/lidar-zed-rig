"""Estimate the ZED's capture delay (arrival time - capture time) from recordings with motion.

For a candidate delay d, image i was captured at t = arrival_i - d. For each direction, take the
LiDAR depth-edge measurement nearest in time to t (within one spin), project it with the calibration
and measure the distance to the image's edges. While the rig moves, a wrong d pairs the image with
points from another pose and the edges drift apart, so the d with the lowest error is the delay.
The hold-still start carries no information and is skipped.

  lidarzed-delay sessions/calib01 sessions/calib05 --T calibration/2026-10-28/T_lidar_camera.json --plot delay.png
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from ouster.sdk import core
from scipy.ndimage import map_coordinates

from ..camera import lens_from_session
from ..lidar import column_host_ns, frames, open_osf
from ..transforms import load_T
from .edges import depth_edge_mask, image_edge_distance, project, to_camera

WINDOW_NS = 50_000_000  # one spin: each direction contributes its measurement nearest the capture time
CLIP_PX = 20.0


def lidar_edges_with_times(session):
    """All depth-edge points of every complete scan (LiDAR frame), with host-clock measurement times."""
    src, info = open_osf(session / "lidar.osf")
    lut = core.XYZLut(info)
    pts, times = [], []
    for fr in frames(src):
        col_ns = column_host_ns(fr)
        rng = fr.field(core.ChanField.RANGE)
        rows, cols = np.nonzero(depth_edge_mask(info, rng))
        pts.append(lut(rng)[rows, cols])
        times.append(col_ns[cols])
    src.close()
    P, t = np.vstack(pts), np.concatenate(times)
    order = np.argsort(t)
    return P[order], t[order]


def delay_curve(session, T, delays_ms, skip_start_s=11):
    K, dist = lens_from_session(session)
    ims = [(int(r["host_ns"]), r["file"]) for r in csv.DictReader(open(session / "zed" / "frames.csv"))]
    P, t = lidar_edges_with_times(session)
    Pc_all = to_camera(P, T)
    t0 = ims[0][0] + skip_start_s * 1e9
    errs = np.full((len(ims), len(delays_ms)), np.nan)
    for i, (ns, name) in enumerate(ims):
        if ns < t0 or i % 2:  # every other image is plenty
            continue
        dt_img = image_edge_distance(cv2.imread(str(session / "zed" / "left" / name), cv2.IMREAD_GRAYSCALE))
        for j, d in enumerate(delays_ms):
            tc = ns - d * 1e6
            lo, hi = np.searchsorted(t, [tc - WINDOW_NS, tc + WINDOW_NS])
            uv, _ = project(Pc_all[lo:hi], K, dist, dt_img.shape, margin=2)
            if len(uv) < 80:
                continue
            e = map_coordinates(dt_img, [uv[:, 1], uv[:, 0]], order=1)
            errs[i, j] = np.mean(np.minimum(e, CLIP_PX))
    full = ~np.isnan(errs).any(axis=1)  # compare all delays on the same images
    return errs[full].mean(axis=0), int(full.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--T", required=True)
    ap.add_argument("--min", type=float, default=-50, help="ms [-50]")
    ap.add_argument("--max", type=float, default=200, help="ms [200]")
    ap.add_argument("--step", type=float, default=5, help="ms [5]")
    ap.add_argument("--skip-start", type=float, default=11, help="seconds of hold-still to skip [11]")
    ap.add_argument("--plot", help="save the error-vs-delay curve as PNG")
    args = ap.parse_args()

    T = load_T(args.T)
    delays = np.arange(args.min, args.max + 1e-9, args.step)
    curves = {}
    for s in args.sessions:
        curve, n = delay_curve(s, T, delays, args.skip_start)
        curves[s.name] = curve
        print(f"{s.name}: {n} images used, best delay {delays[np.argmin(curve)]:+.0f} ms "
              f"(error {curve.min():.2f} px vs {curve[np.argmin(np.abs(delays))]:.2f} px at 0 ms)")

    mean = np.mean([c - c.min() for c in curves.values()], axis=0)
    k = int(np.argmin(mean))
    best = delays[k]
    if 0 < k < len(delays) - 1:  # parabola through the minimum and its neighbours
        y0, y1, y2 = mean[k - 1:k + 2]
        best = delays[k] + 0.5 * args.step * (y0 - y2) / (y0 - 2 * y1 + y2)
    print(f"\nCOMBINED best delay: {best:+.1f} ms (camera image arrives this much after capture)")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        for name, c in curves.items():
            ax.plot(delays, c - c.min(), label=name, alpha=0.7)
        ax.plot(delays, mean, "k", lw=2.5, label="mean")
        ax.axvline(best, color="k", ls="--")
        ax.set_xlabel("assumed camera delay (ms)"); ax.set_ylabel("edge error above minimum (px)")
        ax.set_title(f"Camera delay estimate: {best:+.1f} ms"); ax.legend(); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(args.plot, dpi=90)


if __name__ == "__main__":
    main()
