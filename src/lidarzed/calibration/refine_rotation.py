"""Refine only the camera rotation, with the translation held at a physically measured value.

Targetless calibration fits translation and rotation together; when the translation drifts, the
rotation compensates. This keeps a measured translation and fits the 3 rotation angles by minimizing
how far projected LiDAR depth edges land from image edges, over all given sessions.

  lidarzed-refine-rotation sessions/calib0[1-5] --start calibration/2026-10-28/init_T_lidar_camera.json \
      --t 0.078 -0.060 -0.024 --out T_rotfit.json
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from ..transforms import load_T, save_T
from .edge_score import session_data
from .edges import project

CLIP_PX = 15.0  # edges farther than this count as "missed", so outliers don't dominate


def cost(R, t, data):
    per_session = []
    for P, K, dist, dt in data:
        uv, _ = project((P - t) @ R, K, dist, dt.shape, margin=2)  # camera coords: R^T (p - t)
        e = map_coordinates(dt, [uv[:, 1], uv[:, 0]], order=1)     # bilinear: smooth for the optimizer
        per_session.append(np.mean(np.minimum(e, CLIP_PX)))
    return float(np.mean(per_session)), per_session


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--start", required=True, help="calibration JSON whose rotation starts the search")
    ap.add_argument("--t", nargs=3, type=float, required=True, help="fixed translation (m), sensor frame")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    t = np.array(args.t)
    R0 = load_T(args.start)[:3, :3]
    data = [session_data(s) for s in args.sessions]

    def f(rv_deg):
        return cost(R0 @ Rotation.from_rotvec(np.radians(rv_deg)).as_matrix(), t, data)[0]

    c0, s0 = cost(R0, t, data)
    res = minimize(f, np.zeros(3), method="Powell", options={"xtol": 1e-3, "ftol": 1e-5, "maxfev": 600})
    R = R0 @ Rotation.from_rotvec(np.radians(res.x)).as_matrix()
    c1, s1 = cost(R, t, data)
    print(f"start rotation: mean clipped edge distance {c0:.3f} px  per session {np.round(s0, 2).tolist()}")
    print(f"fitted        : mean clipped edge distance {c1:.3f} px  per session {np.round(s1, 2).tolist()}")
    print(f"rotation change (about camera x/y/z): {np.round(res.x, 3).tolist()} deg ({np.linalg.norm(res.x):.2f} deg total)")
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    save_T(args.out, T, f"Rotation fitted to edges over {[s.name for s in args.sessions]}, translation fixed. Start: {args.start}")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
