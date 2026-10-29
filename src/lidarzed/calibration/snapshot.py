"""Grab one ZED frame and a few OS1 scans right now, to check a scene before recording.

Saves <out>/snapshot.jpg (raw left image) and <out>/snapshot_points.npy (sensor-frame XYZ, N x 3).
  lidarzed-snapshot --sensor os-XXXXXXXXXXXX.local --out snapshot/
"""
import argparse
import itertools
from pathlib import Path

import cv2
import numpy as np
from ouster.sdk import core, open_source

from ..camera import RESOLUTIONS, open_zed


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--sensor", required=True, help="OS1 hostname or IP")
    ap.add_argument("--scans", type=int, default=5)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    cap, _ = open_zed(None, "720")
    if cap is None:
        raise SystemExit("ZED not available")
    for _ in range(10):  # let exposure settle
        cap.read()
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("ZED frame grab failed")
    cv2.imwrite(str(args.out / "snapshot.jpg"), frame[:, : RESOLUTIONS["720"][0] // 2])

    src = open_source(args.sensor)
    info = src.sensor_info[0] if isinstance(src.sensor_info, list) else src.sensor_info
    lut = core.XYZLut(info)
    pts = []
    for item in itertools.islice(src, args.scans + 1):
        fr = item[0]
        if fr is None or not fr.complete():
            continue
        rng = fr.field(core.ChanField.RANGE)
        pts.append(lut(rng)[rng > 0])
    src.close()
    np.save(args.out / "snapshot_points.npy", np.vstack(pts).astype(np.float32))
    print(f"saved {args.out}/snapshot.jpg and {sum(len(p) for p in pts):,} points from {len(pts)} scans")


if __name__ == "__main__":
    main()
