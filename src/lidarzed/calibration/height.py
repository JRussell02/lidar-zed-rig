"""Check the camera's height below the OS1 from a still recording with near horizontal edges.

Targetless calibration constrains height poorly in rooms full of vertical edges. A still recording of
a box top (near, ~1 m) in front of a wall (far) helps: for each candidate height the camera tilt is
re-fitted, and the right height lines up BOTH the near and the far horizontal edges.

  lidarzed-height sessions/height01 --T calibration/2026-10-28/T_lidar_camera.json --plot height.png
"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from ouster.sdk import core
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize_scalar
from scipy.spatial.transform import Rotation

from ..camera import lens_from_session
from ..lidar import frames, open_osf
from ..transforms import load_T
from .edges import depth_edge_mask, image_edge_distance, project

CLIP_PX = 15.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("--T", required=True, help="calibration to start from (rotation, forward/side position)")
    ap.add_argument("--zmin", type=float, default=-70, help="mm [-70]")
    ap.add_argument("--zmax", type=float, default=-10, help="mm [-10]")
    ap.add_argument("--plot")
    args = ap.parse_args()

    s = args.session
    K, dist = lens_from_session(s)
    files = [r["file"] for r in csv.DictReader(open(s / "zed" / "frames.csv"))][::10]
    mean_img = np.mean([cv2.imread(str(s / "zed" / "left" / f), cv2.IMREAD_GRAYSCALE).astype(np.float32)
                        for f in files], axis=0).astype(np.uint8)  # still rig: average out noise
    dt = image_edge_distance(mean_img, horizontal_only=True)

    src, info = open_osf(s / "lidar.osf")
    lut = core.XYZLut(info)
    P = np.vstack([lut(fr.field(core.ChanField.RANGE))[depth_edge_mask(info, fr.field(core.ChanField.RANGE),
                                                                       horizontal_only=True)] for fr in frames(src)])
    src.close()
    T0 = load_T(args.T)
    R0, t0 = T0[:3, :3], T0[:3, 3].copy()

    def err(R, t, band=None):
        uv, z = project((P - t) @ R, K, dist, dt.shape, min_z=0.4, margin=2)
        if band:
            uv = uv[(z >= band[0]) & (z < band[1])]
        if len(uv) < 50:
            return np.nan
        return float(np.mean(np.minimum(map_coordinates(dt, [uv[:, 1], uv[:, 0]], order=1), CLIP_PX)))

    def with_tilt(deg):  # tilt = rotation about the camera's x axis
        return R0 @ Rotation.from_euler("x", deg, degrees=True).as_matrix()

    rows = []
    for z_mm in np.arange(args.zmin, args.zmax + 1e-9, 2.0):
        t = t0.copy(); t[2] = z_mm / 1000
        r = minimize_scalar(lambda a: err(with_tilt(a), t), bounds=(-3, 3), method="bounded", options={"xatol": 0.005})
        R = with_tilt(r.x)
        rows.append((z_mm, r.x, r.fun, err(R, t, (0.4, 1.5)), err(R, t, (1.5, 99))))
    rows = np.array(rows)
    k = int(np.nanargmin(rows[:, 2]))
    best_z = rows[k, 0]
    if 0 < k < len(rows) - 1:
        y0, y1, y2 = rows[k - 1:k + 2, 2]
        best_z = rows[k, 0] + 1.0 * (y0 - y2) / (y0 - 2 * y1 + y2)

    print(f"{len(P):,} horizontal LiDAR edge points")
    print(f"{'height mm':>9s} {'tilt fit':>8s} {'all px':>7s} {'near<1.5m':>9s} {'far':>6s}")
    for z_mm, tilt, e, en, ef in rows:
        print(f"{z_mm:9.0f} {tilt:+8.3f} {e:7.3f} {en:9.3f} {ef:6.3f}{'  <-- best' if z_mm == rows[k, 0] else ''}")
    print(f"\nBEST camera height: {best_z:.1f} mm (sensor z); starting calibration had {t0[2] * 1000:.1f} mm")
    print("Note: curves are often shallow; compare against a ruler measurement before trusting a change.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        ax.plot(rows[:, 0], rows[:, 2] - np.nanmin(rows[:, 2]), "k-o", ms=3, label="all horizontal edges")
        ax.plot(rows[:, 0], rows[:, 3] - np.nanmin(rows[:, 3]), "-", alpha=.7, label="near (< 1.5 m)")
        ax.plot(rows[:, 0], rows[:, 4] - np.nanmin(rows[:, 4]), "-", alpha=.7, label="far (> 1.5 m)")
        ax.axvline(best_z, color="k", ls="--")
        ax.set_xlabel("camera height relative to OS1 base (mm, sensor z)"); ax.set_ylabel("edge error above minimum (px)")
        ax.set_title(f"Camera height test: best {best_z:.1f} mm"); ax.legend(); ax.grid(alpha=.3)
        fig.tight_layout(); fig.savefig(args.plot, dpi=90)


if __name__ == "__main__":
    main()
