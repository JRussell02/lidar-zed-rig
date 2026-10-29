"""Convert lidarzed-record sessions into ROS 2 bags for direct_visual_lidar_calibration.

The calibration tool uses the FIRST image in each bag and builds its dense LiDAR cloud in the frame of
the FIRST LiDAR scan, so each bag starts with one image taken while the rig was held still, then
every complete LiDAR scan from that moment on (the tool's CT-ICP integrates them while you move).

Per bag:
  /zed/left/camera_info            sensor_msgs/CameraInfo      (factory calibration, raw left image)
  /zed/left/image_raw/compressed   sensor_msgs/CompressedImage (the raw JPEG, unflipped)
  /os1/points                      sensor_msgs/PointCloud2     x y z intensity(SIGNAL) reflectivity t
                                   in the Ouster SENSOR frame; t = seconds since the scan started

  lidarzed-make-bags sessions/calib01 sessions/calib02 ... --out work/bags --max-seconds 60
"""
import argparse
import csv
import json
import shutil
from pathlib import Path

import numpy as np
from ouster.sdk import core
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from ..camera import lens_from_session
from ..lidar import column_host_ns, frames, open_osf

TS = get_typestore(Stores.ROS2_HUMBLE)
PointCloud2 = TS.types["sensor_msgs/msg/PointCloud2"]
PointField = TS.types["sensor_msgs/msg/PointField"]
CompressedImage = TS.types["sensor_msgs/msg/CompressedImage"]
CameraInfo = TS.types["sensor_msgs/msg/CameraInfo"]
RegionOfInterest = TS.types["sensor_msgs/msg/RegionOfInterest"]
Header = TS.types["std_msgs/msg/Header"]
Time = TS.types["builtin_interfaces/msg/Time"]

POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
                        ("reflectivity", "<f4"), ("t", "<f4")])
FIELDS = [PointField(name=n, offset=POINT_DTYPE.fields[n][1], datatype=7, count=1)  # 7 = FLOAT32
          for n in POINT_DTYPE.names]


def header(ns, frame_id):
    return Header(stamp=Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000)), frame_id=frame_id)


def camera_info(K, dist, width, height, ns):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return CameraInfo(
        header=header(ns, "zed_left"), height=height, width=width, distortion_model="plumb_bob",
        d=np.asarray(dist, dtype=np.float64), k=K.astype(np.float64).ravel(), r=np.eye(3).ravel(),
        p=np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0], dtype=np.float64),
        binning_x=0, binning_y=0, roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


def convert(session, out_dir, max_seconds=30, min_hold_margin=0.5):
    meta = json.loads((session / "meta.json").read_text())
    K, dist = lens_from_session(session)
    with open(session / "zed" / "frames.csv") as f:
        images = [(int(r["host_ns"]), r["file"]) for r in csv.DictReader(f)]
    img_ns = np.array([t for t, _ in images])

    src, info = open_osf(session / "lidar.osf")
    xyz_lut = core.XYZLut(info)  # Ouster SENSOR frame (origin at the base center)

    bag_path = out_dir / session.name
    if bag_path.exists():
        shutil.rmtree(bag_path)
    n_scans, ref = 0, None
    with Writer(bag_path) as bag:
        c_info = bag.add_connection("/zed/left/camera_info", CameraInfo.__msgtype__, typestore=TS)
        c_img = bag.add_connection("/zed/left/image_raw/compressed", CompressedImage.__msgtype__, typestore=TS)
        c_pts = bag.add_connection("/os1/points", PointCloud2.__msgtype__, typestore=TS)

        for frame in frames(src):
            col_ns = column_host_ns(frame)
            start_ns = int(col_ns[col_ns > 0].min())
            if ref is None and start_ns < img_ns[0]:
                continue  # recorded before the hold-still window (camera loop) began

            if ref is None:
                # Reference = first complete scan in the hold-still window, paired with the nearest image
                i = int(np.argmin(np.abs(img_ns - start_ns)))
                hold_end = img_ns[0] + meta.get("hold_s", 0) * 1e9
                if img_ns[i] > hold_end - min_hold_margin * 1e9:
                    raise SystemExit(f"{session.name}: first LiDAR scan is after the hold-still window; "
                                     "re-record with a longer --hold")
                ref = {"scan_start_ns": start_ns, "image": images[i][1], "image_ns": int(img_ns[i]),
                       "dt_ms": (img_ns[i] - start_ns) / 1e6}
                jpeg = (session / "zed" / "left" / images[i][1]).read_bytes()
                bag.write(c_info, int(img_ns[i]), TS.serialize_cdr(
                    camera_info(K, dist, meta["zed"]["width_per_eye"], meta["zed"]["height"], img_ns[i]),
                    CameraInfo.__msgtype__))
                bag.write(c_img, int(img_ns[i]), TS.serialize_cdr(
                    CompressedImage(header=header(img_ns[i], "zed_left"), format="jpeg",
                                    data=np.frombuffer(jpeg, dtype=np.uint8)), CompressedImage.__msgtype__))

            if max_seconds and start_ns - ref["scan_start_ns"] > max_seconds * 1e9:
                break

            rng = frame.field(core.ChanField.RANGE)
            valid = rng > 0
            xyz = xyz_lut(rng)[valid]
            pts = np.empty(len(xyz), dtype=POINT_DTYPE)
            pts["x"], pts["y"], pts["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
            pts["intensity"] = frame.field(core.ChanField.SIGNAL)[valid]
            pts["reflectivity"] = frame.field(core.ChanField.REFLECTIVITY)[valid]
            pts["t"] = np.broadcast_to((col_ns - start_ns) / 1e9, rng.shape)[valid]
            msg = PointCloud2(header=header(start_ns, "os1_sensor"), height=1, width=len(pts), fields=FIELDS,
                              is_bigendian=False, point_step=POINT_DTYPE.itemsize,
                              row_step=POINT_DTYPE.itemsize * len(pts),
                              data=np.frombuffer(pts.tobytes(), dtype=np.uint8), is_dense=True)
            bag.write(c_pts, max(start_ns, int(img_ns[0])), TS.serialize_cdr(msg, PointCloud2.__msgtype__))
            n_scans += 1
    src.close()

    if ref is None:
        raise SystemExit(f"{session.name}: no complete LiDAR scans")
    print(f"{session.name}: {n_scans} scans, reference image {ref['image']} "
          f"({ref['dt_ms']:+.0f} ms from first scan) -> {bag_path}")
    (bag_path / "vlcal_source.json").write_text(json.dumps({"session": str(session), **ref}, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sessions", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="folder for the bags (one subfolder per session)")
    ap.add_argument("--max-seconds", type=float, default=30, help="LiDAR seconds per bag after the reference [30]")
    ap.add_argument("--min-hold-margin", type=float, default=0.5,
                    help="reference image must be at least this long before the hold ends [0.5 s]")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    for s in args.sessions:
        convert(s, args.out, args.max_seconds, args.min_hold_margin)


if __name__ == "__main__":
    main()
