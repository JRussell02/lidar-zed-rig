"""Write a starting guess into a preprocessed folder's calib.json for the tool's `calibrate` step.

The tool reads results.init_T_lidar_camera = [tx, ty, tz, qx, qy, qz, qw]: the camera's pose in the
LiDAR (Ouster sensor) frame, in meters. Our transform JSON stores exactly that.

  lidarzed-init-guess work/pre --init calibration/2026-10-28/init_T_lidar_camera.json
"""
import argparse
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pre_dir", type=Path, help="folder written by `preprocess` (contains calib.json)")
    ap.add_argument("--init", type=Path, required=True, help="transform JSON (translation_m + quaternion_xyzw)")
    args = ap.parse_args()

    init = json.loads(args.init.read_text())
    values = list(init["translation_m"]) + list(init["quaternion_xyzw"])
    calib_path = args.pre_dir / "calib.json"
    calib = json.loads(calib_path.read_text())
    calib.setdefault("results", {})["init_T_lidar_camera"] = values
    calib_path.write_text(json.dumps(calib, indent=2))
    print(f"{calib_path}: init_T_lidar_camera = {values}")


if __name__ == "__main__":
    main()
