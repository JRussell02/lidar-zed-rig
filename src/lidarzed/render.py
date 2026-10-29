"""Quick headless render of a colored PLY (x y z red green blue) from a virtual camera, for checking.

The world frame is the first LiDAR scan's sensor frame (+X forward, +Y left, +Z up).
Usage: lidarzed-render processed/X_color.ply --out view.jpg [--eye -1.5 0 0.6 --look 3 0 0]
"""
import argparse

import cv2
import numpy as np


def read_ply(path):
    with open(path, "rb") as f:
        n = 0
        while True:
            line = f.readline().decode().strip()
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            if line == "end_header":
                break
        rec = np.frombuffer(f.read(), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                             ("r", "u1"), ("g", "u1"), ("b", "u1")], count=n)
    return np.stack([rec["x"], rec["y"], rec["z"]], 1).astype(np.float64), np.stack([rec["b"], rec["g"], rec["r"]], 1)


def render(xyz, bgr, eye, look, w=1280, h=720, fov_deg=80, splat=2):
    f = (w / 2) / np.tan(np.radians(fov_deg / 2))
    fwd = np.asarray(look, float) - eye; fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    p = xyz - eye
    x, y, z = p @ right, p @ down, p @ fwd
    m = z > 0.2
    u = (f * x[m] / z[m] + w / 2).astype(int); v = (f * y[m] / z[m] + h / 2).astype(int)
    z, c = z[m], bgr[m]
    ok = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z, c = u[ok], v[ok], z[ok], c[ok]
    order = np.argsort(-z)  # far first, near overwrite
    img = np.full((h, w, 3), 30, np.uint8)
    for du in range(splat):
        for dv in range(splat):
            uu = np.clip(u[order] + du, 0, w - 1); vv = np.clip(v[order] + dv, 0, h - 1)
            img[vv, uu] = c[order]
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ply")
    ap.add_argument("--out", required=True)
    ap.add_argument("--eye", nargs=3, type=float, default=[-1.5, 0.0, 0.6])
    ap.add_argument("--look", nargs=3, type=float, default=[3.0, 0.0, 0.0])
    ap.add_argument("--fov", type=float, default=80)
    args = ap.parse_args()
    xyz, bgr = read_ply(args.ply)
    cv2.imwrite(args.out, render(xyz, bgr, np.array(args.eye), args.look, fov_deg=args.fov))
    print(f"{args.out}: {len(xyz):,} points rendered")


if __name__ == "__main__":
    main()
