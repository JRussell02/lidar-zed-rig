"""Evaluate and merge room scans (Ouster SLAM clouds or lidarzed-colorize PLYs; colors are kept when merging).

Commands:
  drift  TRAJ.csv [...]                 loop-closure drift from save_trajectory CSVs
  align  REF.ply OTHER.ply [...]        ICP-align scans to REF, report scan-to-scan agreement
         [--merge OUT.ply] [--yaw-search]
  refs   MEASUREMENTS.csv               compare tape measurements with scan measurements

Examples:
  lidarzed-eval drift processed/A1_traj.csv processed/A2_traj.csv
  lidarzed-eval align processed/A1.ply processed/A2.ply --merge merged.ply
  lidarzed-eval refs measurements.csv
"""
import argparse
import csv
import sys

import numpy as np
from scipy.spatial import cKDTree


# ---------- PLY I/O (Ouster SLAM: x,y,z + float attributes; lidarzed-colorize: x,y,z + uchar red,green,blue) ----------

PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
             "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1", "ushort": "<u2", "uint16": "<u2",
             "short": "<i2", "int16": "<i2", "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}


def read_ply_rgb(path):
    """Return (Nx3 float64 xyz, Nx3 float64 rgb or None) from a PLY point cloud."""
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            header.append(line)
            if line == "end_header":
                break
        fmt = next(l for l in header if l.startswith("format")).split()[1]
        count = int(next(l for l in header if l.startswith("element vertex")).split()[2])
        props = [l.split()[1:3] for l in header if l.startswith("property")]
        names = [n for _, n in props]
        if fmt == "ascii":
            data = np.loadtxt(f, max_rows=count, ndmin=2)
            col = lambda n: data[:, names.index(n)]
        else:
            if any(t not in PLY_TYPES for t, _ in props):
                raise SystemExit(f"{path}: unsupported property type")
            data = np.fromfile(f, dtype=[(n, PLY_TYPES[t]) for t, n in props], count=count)
            col = lambda n: data[n]
    pts = np.stack([col(c) for c in ("x", "y", "z")], 1).astype(np.float64)
    rgb = np.stack([col(c) for c in ("red", "green", "blue")], 1).astype(np.float64) \
        if {"red", "green", "blue"} <= set(names) else None
    ok = np.isfinite(pts).all(axis=1)
    return pts[ok], (rgb[ok] if rgb is not None else None)


def read_ply(path):
    """Return Nx3 float64 xyz from a PLY point cloud."""
    return read_ply_rgb(path)[0]


def write_ply(path, pts, rgb=None):
    """Binary PLY: x y z, plus uchar red green blue if colors are given."""
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if rgb is not None:
        fields += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    rec = np.empty(len(pts), dtype=fields)
    rec["x"], rec["y"], rec["z"] = pts.T
    if rgb is not None:
        rec["red"], rec["green"], rec["blue"] = np.clip(np.round(rgb), 0, 255).astype(np.uint8).T
    props = "".join(f"property {'float' if t == '<f4' else 'uchar'} {n}\n" for n, t in fields)
    with open(path, "wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {len(rec)}\n{props}end_header\n".encode())
        f.write(rec.tobytes())


# ---------- point cloud helpers ----------

def voxel_down(pts, size, rgb=None):
    """Average points within each voxel of the given size (meters); with rgb, return (points, colors)."""
    keys = np.floor(pts / size).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    out = np.zeros((len(counts), 3))
    np.add.at(out, inv, pts)
    if rgb is None:
        return out / counts[:, None]
    col = np.zeros((len(counts), 3))
    np.add.at(col, inv, rgb)
    return out / counts[:, None], col / counts[:, None]


def transform(pts, T):
    return pts @ T[:3, :3].T + T[:3, 3]


def rot_z(deg):
    a = np.radians(deg)
    T = np.eye(4)
    T[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    return T


def kabsch(src, dst):
    """Rigid transform minimizing ||R src + t - dst||."""
    cs, cd = src.mean(0), dst.mean(0)
    U, _, Vt = np.linalg.svd((src - cs).T @ (dst - cd))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, cd - R @ cs
    return T


def inlier_fraction(src, tree, max_d):
    d, _ = tree.query(src, distance_upper_bound=max_d)
    return np.isfinite(d).mean()


def icp(src, tgt_tree, tgt, T0, schedule=(1.0, 0.5, 0.25, 0.1, 0.05), iters=20):
    """Point-to-point ICP with a shrinking correspondence distance."""
    T = T0.copy()
    for max_d in schedule:
        for _ in range(iters):
            moved = transform(src, T)
            d, i = tgt_tree.query(moved, distance_upper_bound=max_d)
            ok = np.isfinite(d)
            if ok.sum() < 50:
                break
            dT = kabsch(moved[ok], tgt[i[ok]])
            T = dT @ T
            if np.linalg.norm(dT[:3, 3]) < 1e-5 and np.allclose(dT[:3, :3], np.eye(3), atol=1e-7):
                break
    return T


def dist_stats(d, overlap_d):
    o = d[d < overlap_d]
    if len(o) == 0:
        return dict(overlap=0.0)
    return dict(overlap=len(o) / len(d), mean=o.mean(), median=np.median(o),
                p95=np.percentile(o, 95), rms=np.sqrt((o ** 2).mean()))


# ---------- commands ----------

def cmd_drift(args):
    print(f"{'trajectory':40s} {'path m':>8s} {'start-end m':>12s} {'drift %':>8s} {'Δz m':>7s}")
    for path in args.traj:
        with open(path) as f:
            rows = [r for r in csv.DictReader(f)]
        p = np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])
        length = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        gap = np.linalg.norm(p[-1] - p[0])
        print(f"{path:40s} {length:8.1f} {gap:12.3f} {100 * gap / max(length, 1e-9):8.2f} {p[-1, 2] - p[0, 2]:7.3f}")
    print("\nOnly meaningful if each scan ended exactly on its start mark.")


def cmd_align(args):
    ref_full, ref_rgb = read_ply_rgb(args.ref)
    ref = voxel_down(ref_full, args.voxel)
    ref_tree = cKDTree(ref)
    ref_eval = voxel_down(ref_full, args.eval_voxel)
    ref_eval_tree = cKDTree(ref_eval)
    print(f"Reference {args.ref}: {len(ref_full):,} pts")

    merged, merged_rgb = [ref_full], [ref_rgb]
    summary = []
    for path in args.others:
        full, rgb = read_ply_rgb(path)
        src = voxel_down(full, args.voxel)

        T0 = np.eye(4)
        if args.yaw_search:
            # coarse heading search in case the scans didn't start with the same heading
            to_origin, to_ref = np.eye(4), np.eye(4)
            to_origin[:3, 3], to_ref[:3, 3] = -src.mean(0), ref.mean(0)
            candidates = [to_ref @ rot_z(yaw) @ to_origin for yaw in range(0, 360, 10)]
            T0 = max(candidates, key=lambda c: inlier_fraction(transform(src, c), ref_tree, 0.5))
        T = icp(src, ref_tree, ref, T0)

        ev = transform(voxel_down(full, args.eval_voxel), T)
        d, _ = ref_eval_tree.query(ev)
        s = dist_stats(d, args.overlap)
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        print(f"\n{path}: {len(full):,} pts")
        print(f"  ICP correction: translation {np.linalg.norm(T[:3, 3]):.3f} m, yaw {yaw:+.2f}°")
        if s["overlap"] == 0:
            print("  NO OVERLAP: alignment failed (try --yaw-search)")
            continue
        print(f"  overlap (<{args.overlap * 100:.0f} cm): {s['overlap'] * 100:.1f}%")
        print(f"  cloud-to-cloud distance: mean {s['mean'] * 100:.1f} cm, median {s['median'] * 100:.1f} cm, "
              f"p95 {s['p95'] * 100:.1f} cm, RMS {s['rms'] * 100:.1f} cm")
        if s["overlap"] < args.min_overlap:
            print(f"  ⚠ overlap below {args.min_overlap * 100:.0f}%: probably misaligned, NOT merged "
                  "(try --yaw-search, or lower --min-overlap if the scans cover different areas)")
            continue
        summary.append((path, s))
        merged.append(transform(full, T))
        merged_rgb.append(rgb)
        np.savetxt(path.rsplit(".", 1)[0] + "_to_ref.txt", T, fmt="%.8f",
                   header=f"4x4 transform aligning {path} to {args.ref}")

    if args.merge:
        colored = all(c is not None for c in merged_rgb)
        if not colored and any(c is not None for c in merged_rgb):
            print("\nSome scans have no colors: writing the merged cloud without colors")
        if colored:
            out, out_rgb = voxel_down(np.vstack(merged), args.merge_voxel, np.vstack(merged_rgb))
        else:
            out, out_rgb = voxel_down(np.vstack(merged), args.merge_voxel), None
        write_ply(args.merge, out, out_rgb)
        print(f"\nMerged {len(merged)} scans -> {args.merge} ({len(out):,} pts, {args.merge_voxel * 100:.0f} cm voxels"
              f"{', colored' if colored else ''})")

    if len(summary) > 1:
        rms = np.array([s["rms"] for _, s in summary])
        print(f"\nAcross {len(summary)} scans: RMS {rms.mean() * 100:.1f} ± {rms.std() * 100:.1f} cm")


def cmd_refs(args):
    with open(args.csv) as f:
        rows = [r for r in csv.DictReader(f) if r.get("scan_m", "").strip()]
    if not rows:
        raise SystemExit("No rows with scan_m filled in.")
    errs = []
    print(f"{'measurement':30s} {'tape m':>8s} {'scan m':>8s} {'error cm':>9s} {'error %':>8s}")
    for r in rows:
        tape, scan = float(r["tape_m"]), float(r["scan_m"])
        e = scan - tape
        errs.append(e)
        print(f"{r['name']:30s} {tape:8.3f} {scan:8.3f} {e * 100:+9.1f} {100 * e / tape:+8.2f}")
    errs = np.array(errs)
    print(f"\nmean error {errs.mean() * 100:+.1f} cm (bias), mean |error| {np.abs(errs).mean() * 100:.1f} cm, "
          f"max |error| {np.abs(errs).max() * 100:.1f} cm")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("drift", help="start/end drift from trajectory CSVs")
    p.add_argument("traj", nargs="+")
    p.set_defaults(func=cmd_drift)

    p = sub.add_parser("align", help="align scans to a reference and report agreement")
    p.add_argument("ref")
    p.add_argument("others", nargs="+")
    p.add_argument("--merge", help="write merged point cloud to this PLY")
    p.add_argument("--yaw-search", action="store_true", help="search heading before ICP")
    p.add_argument("--voxel", type=float, default=0.05, help="ICP downsample voxel (m) [0.05]")
    p.add_argument("--eval-voxel", type=float, default=0.02, help="evaluation voxel (m) [0.02]")
    p.add_argument("--overlap", type=float, default=0.10, help="max distance counted as overlap (m) [0.10]")
    p.add_argument("--merge-voxel", type=float, default=0.02, help="merged cloud voxel (m) [0.02]")
    p.add_argument("--min-overlap", type=float, default=0.5, help="skip scans below this overlap fraction [0.5]")
    p.set_defaults(func=cmd_align)

    p = sub.add_parser("refs", help="compare tape measurements to scan measurements")
    p.add_argument("csv")
    p.set_defaults(func=cmd_refs)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
