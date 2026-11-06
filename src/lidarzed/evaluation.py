"""Evaluate and merge room scans (Ouster SLAM clouds or lidarzed-colorize PLYs; colors are kept when merging).

Commands:
  path   SLAM.osf [...] [--csv OUT]     check a SLAM result: path height, tilt, jumps (did SLAM lose track?)
  drift  TRAJ.csv [...]                 loop-closure drift from save_trajectory CSVs
  align  REF.ply OTHER.ply [...]        ICP-align scans to REF, report scan-to-scan agreement
         [--merge OUT.ply] [--yaw-search | --global]
  refs   MEASUREMENTS.csv               compare tape measurements with scan measurements

Examples:
  lidarzed-eval path processed/A1_slam.osf
  lidarzed-eval drift processed/A1_traj.csv processed/A2_traj.csv
  lidarzed-eval align processed/A1.ply processed/A2.ply --merge merged.ply
  lidarzed-eval align processed/A1.ply processed/B1.ply --global --merge merged.ply   # different start spots
  lidarzed-eval refs measurements.csv
"""
import argparse
import csv
import sys

import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.signal import fftconvolve
from scipy.spatial import cKDTree

GRAY = 128          # color of points no camera saw (lidarzed-colorize --all-points)


# ---------- PLY I/O (Ouster SLAM: x,y,z + float attributes; lidarzed-colorize: x,y,z + uchar red,green,blue
#            [+ uchar colored with --all-points]) ----------

PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8",
             "uchar": "u1", "uint8": "u1", "char": "i1", "int8": "i1", "ushort": "<u2", "uint16": "<u2",
             "short": "<i2", "int16": "<i2", "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}


def read_ply_colored(path):
    """Return (Nx3 float64 xyz, Nx3 float64 rgb or None, colored bool per point or None) from a PLY."""
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
    colored = (col("colored") > 0) if rgb is not None and "colored" in names else None
    ok = np.isfinite(pts).all(axis=1)
    return pts[ok], (rgb[ok] if rgb is not None else None), (colored[ok] if colored is not None else None)


def read_ply_rgb(path):
    """Return (Nx3 float64 xyz, Nx3 float64 rgb or None) from a PLY point cloud."""
    return read_ply_colored(path)[:2]


def read_ply(path):
    """Return Nx3 float64 xyz from a PLY point cloud."""
    return read_ply_rgb(path)[0]


def write_ply(path, pts, rgb=None, colored=None):
    """Binary PLY: x y z, plus uchar red green blue if colors are given, plus uchar colored if given."""
    fields = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if rgb is not None:
        fields += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    if colored is not None:
        fields += [("colored", "u1")]
    rec = np.empty(len(pts), dtype=fields)
    rec["x"], rec["y"], rec["z"] = pts.T
    if rgb is not None:
        rec["red"], rec["green"], rec["blue"] = np.clip(np.round(rgb), 0, 255).astype(np.uint8).T
    if colored is not None:
        rec["colored"] = colored
    props = "".join(f"property {'float' if t == '<f4' else 'uchar'} {n}\n" for n, t in fields)
    with open(path, "wb") as f:
        f.write(f"ply\nformat binary_little_endian 1.0\nelement vertex {len(rec)}\n{props}end_header\n".encode())
        f.write(rec.tobytes())


# ---------- point cloud helpers ----------

def voxel_down(pts, size, rgb=None, colored=None):
    """Average points within each voxel of the given size (meters); with rgb, return (points, colors).

    With colored (bool per point), colors are averaged over the colored points only and
    (points, colors, colored per voxel) is returned; voxels without any colored point get 0.
    """
    q = np.floor(pts / size).astype(np.int64) + (1 << 20)          # 21 bits per axis packed in one int64
    _, inv, counts = np.unique((q[:, 0] << 42) | (q[:, 1] << 21) | q[:, 2], return_inverse=True, return_counts=True)
    inv = inv.ravel()
    sums = lambda v: np.stack([np.bincount(inv, weights=v[:, j], minlength=len(counts)) for j in range(3)], 1)
    out = sums(pts)
    if rgb is None:
        return out / counts[:, None]
    w = np.ones(len(pts)) if colored is None else colored.astype(float)
    nc = np.bincount(inv, weights=w, minlength=len(counts))
    col = sums(rgb * w[:, None]) / np.maximum(nc, 1)[:, None]
    return (out / counts[:, None], col) if colored is None else (out / counts[:, None], col, nc > 0)


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


# ---------- global registration (scans that didn't start on the same spot/heading) ----------

def normals(pts, k=16):
    """Unit surface normal of every point from the PCA of its k nearest neighbors."""
    _, idx = cKDTree(pts).query(pts, k)
    nb = pts[idx] - pts[idx].mean(axis=1, keepdims=True)
    return np.linalg.eigh(np.einsum("nki,nkj->nij", nb, nb))[1][:, :, 0]


def rot_between(a, b):
    """Smallest rotation taking unit vector a onto unit vector b."""
    v, c = np.cross(a, b), a @ b
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * ((1 - c) / s ** 2)


def up_direction(n, hint=(0.0, 0.0, 1.0), max_deg=30, tol_deg=5):
    """True up from surface normals, or None: the direction most normals within max_deg of hint share.

    Floor, ceiling and table tops all face along true up, so their normals pile up in one direction;
    walls and clutter spread out. Taking that pile's direction (not the mean of everything near hint)
    keeps sloped and noisy surfaces from tilting the result.
    """
    hint = np.asarray(hint, float) / np.linalg.norm(hint)
    n = n * np.where(n @ hint < 0, -1.0, 1.0)[:, None]           # ceilings face down: flip to hint's side
    cand = n[n @ hint > np.cos(np.radians(max_deg))]
    if len(cand) < 100:
        return None
    cos_tol = np.cos(np.radians(tol_deg))
    trial = cand[np.random.default_rng(0).choice(len(cand), min(500, len(cand)), replace=False)]
    best = trial[np.argmax([(cand @ t > cos_tol).sum() for t in trial])]
    m = cand[cand @ best > cos_tol].mean(0)
    return m / np.linalg.norm(m)


def level(pts, n):
    """Transform making floor/ceiling horizontal (up = +z) with the floor at z = 0; also returns a wall mask.

    Up comes from up_direction() within 30° of the cloud's z axis (SLAM's z is the start pose's up);
    the floor is the lowest well-populated height of the horizontal surfaces.
    """
    up = up_direction(n)
    if up is None:
        raise SystemExit("global alignment: no floor or ceiling found")
    horiz = np.abs(n @ up) > np.cos(np.radians(5))
    T = np.eye(4)
    T[:3, :3] = rot_between(up, np.array([0.0, 0.0, 1.0]))
    z = transform(pts[horiz], T)[:, 2]
    hist, edges = np.histogram(z, bins=np.arange(z.min(), z.max() + 0.1, 0.05))
    b = np.argmax(hist >= 0.2 * hist.max())
    T[2, 3] = -np.median(z[(z >= edges[max(b - 1, 0)]) & (z < edges[b + 2])])
    return T, np.abs(n @ up) < 0.3


def rasterize(xy, cell):
    """Binary occupancy image of 2D points; returns (image, xy of pixel [0, 0])."""
    org = xy.min(0)
    ij = np.floor((xy - org) / cell).astype(int)
    img = np.zeros(ij.max(0) + 1)
    img[ij[:, 0], ij[:, 1]] = 1
    return img, org


def global_candidates(src, ref, n_best=4, cell=0.05, yaw_step=2.0):
    """Initial src->ref guesses from leveled floor plans, any start position or heading.

    Both clouds are leveled, then the walls between 0.2 and 2 m above the floor are drawn as 2D
    floor plans and matched over a yaw grid; the shift for each yaw comes from an FFT
    cross-correlation. Returns the n_best 4x4 guesses with distinct headings, best first.
    """
    L_s, wall_s = level(src, normals(src))
    L_r, wall_r = level(ref, normals(ref))

    def wall_xy(p, L, wall):
        q = transform(p, L)
        return q[wall & (q[:, 2] > 0.2) & (q[:, 2] < 2.0), :2]

    ws, wr = wall_xy(src, L_s, wall_s), wall_xy(ref, L_r, wall_r)
    if min(len(ws), len(wr)) < 100:
        raise SystemExit("global alignment: too few wall points")
    ref_img, ref_org = rasterize(wr, cell)
    ref_img = gaussian_filter(ref_img, 1.5)
    found = []
    for yaw in np.arange(0, 360, yaw_step):
        img, org = rasterize(ws @ rot_z(yaw)[:2, :2].T, cell)
        corr = fftconvolve(ref_img, img[::-1, ::-1], mode="full")
        ij = np.array(np.unravel_index(corr.argmax(), corr.shape))
        found.append((corr[tuple(ij)], yaw, ref_org - org + (ij - (np.array(img.shape) - 1)) * cell))
    found.sort(key=lambda f: -f[0])
    picked = []
    for score, yaw, t in found:
        if all(abs((yaw - y + 180) % 360 - 180) >= 10 for _, y, _ in picked):
            picked.append((score, yaw, t))
        if len(picked) == n_best:
            break
    out = []
    for _, yaw, t in picked:
        T2 = rot_z(yaw)
        T2[:2, 3] = t
        out.append(np.linalg.inv(L_r) @ T2 @ L_s)
    return out


def global_align(full, ref_full, voxel=0.1, overlap=0.1):
    """Best src->ref guess: coarse ICP from every global candidate, keep the one with the most overlap."""
    src, ref = voxel_down(full, voxel), voxel_down(ref_full, voxel)
    tree = cKDTree(ref)
    tried = [icp(src, tree, ref, T0, schedule=(1.0, 0.5, 0.25)) for T0 in global_candidates(src, ref)]
    return max(tried, key=lambda T: inlier_fraction(transform(src, T), tree, overlap))


# ---------- SLAM path check ----------

def slam_poses(path, map_every=0):
    """One body-to-world pose per scan (its middle column) from a SLAM'd OSF, as (N, 4, 4).

    With map_every = k, also returns a sparse world-frame map from every k-th scan: (poses, Mx3 points).
    """
    from ouster.sdk import core
    from .lidar import frames, open_osf
    src, info = open_osf(path)
    lut = core.XYZLut(info) if map_every else None
    poses, pts = [], []
    for k, fr in enumerate(frames(src)):
        b2w = np.array(fr.body_to_world, copy=True)               # copies, so the frame can be freed
        rng = np.array(fr.field(core.ChanField.RANGE), copy=True) if map_every and k % map_every == 0 else None
        del fr
        poses.append(b2w[len(b2w) // 2])
        if rng is not None:
            xyz = lut(rng)
            r = np.linalg.norm(xyz, axis=2)
            rows, cols = np.nonzero((rng > 0) & (r > 0.5) & (r < 15))
            rows, cols = rows[::16], cols[::16]                    # plenty for 10 cm voxels
            pts.append(np.einsum("nij,nj->ni", b2w[cols, :3, :3], xyz[rows, cols]) + b2w[cols, :3, 3])
    src.close()
    return (np.array(poses), np.vstack(pts) if pts else np.empty((0, 3))) if map_every else np.array(poses)


def path_report(poses, up=None):
    """Path length, height range, tilt and jumps of a SLAM path, relative to the first pose.

    Heights and tilts are along `up` (true vertical, e.g. from up_direction) if given, else along the
    first pose's up, which is only right if the scan started level.
    """
    up = poses[0, :3, 2] if up is None else up
    p = poses[:, :3, 3] - poses[0, :3, 3]
    h = p @ up
    step = np.linalg.norm(np.diff(p, axis=0), axis=1)
    tilt = np.degrees(np.arccos(np.clip(poses[:, :3, 2] @ up, -1, 1)))
    return dict(scans=len(p), length=step.sum(), z_min=h.min(), z_max=h.max(),
                farthest=np.linalg.norm(p, axis=1).max(), max_jump=step.max(initial=0.0),
                tilt_start=tilt[0], tilt_median=np.median(tilt), tilt_max=tilt.max(),
                end_gap=np.linalg.norm(p[-1]), end_tilt=tilt[-1] - tilt[0])


def dist_stats(d, overlap_d):
    o = d[d < overlap_d]
    if len(o) == 0:
        return dict(overlap=0.0)
    return dict(overlap=len(o) / len(d), mean=o.mean(), median=np.median(o),
                p95=np.percentile(o, 95), rms=np.sqrt((o ** 2).mean()))


# ---------- commands ----------

def cmd_path(args):
    for path in args.osf:
        poses, pts = slam_poses(path, map_every=10)
        if len(poses) < 2 or np.allclose(poses, np.eye(4)):
            print(f"{path}: no SLAM path (use the _slam.osf written by `ouster-cli ... slam ... save`)")
            continue
        d = voxel_down(pts, 0.1)
        up = up_direction(normals(d), max_deg=50)        # tilted starts: true up can be far from the start's
        r = path_report(poses, up)
        ref = "true vertical, from floor/ceiling" if up is not None else \
            "start pose's up: NO floor/ceiling found, wrong if the scan started tilted"
        print(f"{path}: {r['scans']} scans, path {r['length']:.1f} m")
        print(f"  height vs start ({ref}): {r['z_min']:+.2f} .. {r['z_max']:+.2f} m "
              f"(span {r['z_max'] - r['z_min']:.2f} m)")
        print(f"  farthest from start {r['farthest']:.1f} m, largest jump between scans {r['max_jump']:.2f} m")
        print(f"  tilt from vertical: start {r['tilt_start']:.0f}°, median {r['tilt_median']:.0f}°, max {r['tilt_max']:.0f}°")
        print(f"  end vs start: {r['end_gap']:.2f} m, tilt {r['end_tilt']:+.0f}° (only meaningful if you ended on the X)")
        problems = []
        if r["z_max"] - r["z_min"] > args.max_height_span:
            problems.append(f"height span {r['z_max'] - r['z_min']:.1f} m > {args.max_height_span} m")
        if r["max_jump"] > args.max_jump:
            problems.append(f"jump of {r['max_jump']:.2f} m between scans > {args.max_jump} m")
        print(f"  ⚠ SLAM probably lost track: {'; '.join(problems)}" if problems else "  ✅ path looks plausible")
        if args.csv:
            out = args.csv if len(args.osf) == 1 else path.rsplit(".", 1)[0] + "_path.csv"
            np.savetxt(out, poses[:, :3, 3], delimiter=",", header="x,y,z", comments="", fmt="%.4f")
            print(f"  path written to {out} (for `lidarzed-eval drift`)")


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
    ref_full, ref_rgb, ref_colored = read_ply_colored(args.ref)
    ref = voxel_down(ref_full, args.voxel)
    ref_tree = cKDTree(ref)
    ref_eval = voxel_down(ref_full, args.eval_voxel)
    ref_eval_tree = cKDTree(ref_eval)
    print(f"Reference {args.ref}: {len(ref_full):,} pts")

    merged, merged_rgb, merged_colored = [ref_full], [ref_rgb], [ref_colored]
    summary = []
    for path in args.others:
        full, rgb, colored = read_ply_colored(path)
        src = voxel_down(full, args.voxel)

        T0 = np.eye(4)
        if args.global_:
            T0 = global_align(full, ref_full)
        elif args.yaw_search:
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
            print("  NO OVERLAP: alignment failed (try --global)")
            continue
        print(f"  overlap (<{args.overlap * 100:.0f} cm): {s['overlap'] * 100:.1f}%")
        print(f"  cloud-to-cloud distance: mean {s['mean'] * 100:.1f} cm, median {s['median'] * 100:.1f} cm, "
              f"p95 {s['p95'] * 100:.1f} cm, RMS {s['rms'] * 100:.1f} cm")
        if s["overlap"] < args.min_overlap:
            print(f"  ⚠ overlap below {args.min_overlap * 100:.0f}%: probably misaligned, NOT merged "
                  "(try --global, or lower --min-overlap if the scans cover different areas)")
            continue
        summary.append((path, s))
        merged.append(transform(full, T))
        merged_rgb.append(rgb)
        merged_colored.append(colored)
        np.savetxt(path.rsplit(".", 1)[0] + "_to_ref.txt", T, fmt="%.8f",
                   header=f"4x4 transform aligning {path} to {args.ref}")

    if args.merge:
        colored = all(c is not None for c in merged_rgb)
        if not colored and any(c is not None for c in merged_rgb):
            print("\nSome scans have no colors: writing the merged cloud without colors")
        partly = colored and any(c is not None for c in merged_colored)   # some scans kept uncolored points
        out_colored = None
        if partly:
            flags = np.concatenate([np.ones(len(p), bool) if c is None else c
                                    for p, c in zip(merged, merged_colored)])
            out, out_rgb, out_colored = voxel_down(np.vstack(merged), args.merge_voxel, np.vstack(merged_rgb), flags)
            out_rgb[~out_colored] = GRAY
        elif colored:
            out, out_rgb = voxel_down(np.vstack(merged), args.merge_voxel, np.vstack(merged_rgb))
        else:
            out, out_rgb = voxel_down(np.vstack(merged), args.merge_voxel), None
        write_ply(args.merge, out, out_rgb, out_colored)
        print(f"\nMerged {len(merged)} scans -> {args.merge} ({len(out):,} pts, {args.merge_voxel * 100:.0f} cm voxels"
              f"{', colored' if colored else ''})")
        if partly:
            print(f"  {out_colored.mean() * 100:.0f}% colored by some camera; "
                  f"{(~out_colored).sum():,} points no camera saw are gray (colored = 0)")

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

    p = sub.add_parser("path", help="check SLAM paths for lost tracking")
    p.add_argument("osf", nargs="+", help="SLAM'd OSF(s) from `ouster-cli ... slam ... save`")
    p.add_argument("--csv", help="write the path (x,y,z per scan) here; with several files, next to each")
    p.add_argument("--max-height-span", type=float, default=2.0, help="flag paths spanning more height (m) [2.0]")
    p.add_argument("--max-jump", type=float, default=0.5, help="flag jumps between scans above this (m) [0.5]")
    p.set_defaults(func=cmd_path)

    p = sub.add_parser("drift", help="start/end drift from trajectory CSVs")
    p.add_argument("traj", nargs="+")
    p.set_defaults(func=cmd_drift)

    p = sub.add_parser("align", help="align scans to a reference and report agreement")
    p.add_argument("ref")
    p.add_argument("others", nargs="+")
    p.add_argument("--merge", help="write merged point cloud to this PLY")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--yaw-search", action="store_true", help="search heading before ICP (same start spot)")
    g.add_argument("--global", dest="global_", action="store_true",
                   help="floor-plan search over position and heading before ICP (any start spot)")
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
