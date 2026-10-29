"""ZED (1st gen) as a plain USB camera: find/open it on macOS or Linux, read its factory calibration.

The ZED SDK needs an NVIDIA GPU; we don't use it. Over USB (UVC) the ZED delivers the left and right
images side by side in one frame.

Viewer:  lidarzed-view [--res 720] [--upside-down]    (keys: Esc/q quit, s save left/right PNGs)
"""
import argparse
import configparser
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Side-by-side frame sizes (width covers both eyes)
RESOLUTIONS = {"2K": (4416, 1242), "1080": (3840, 1080), "720": (2560, 720), "VGA": (1344, 376)}
# Section names in the ZED factory calibration file (SNxxxxx.conf) for each resolution
CALIB_SECTION = {"2K": "2K", "1080": "FHD", "720": "HD", "VGA": "VGA"}


def find_zed():
    """Index of the ZED's capture device, or None."""
    if sys.platform == "darwin":
        # AVFoundation has no names via OpenCV: the ZED is the camera with a very wide (stereo) frame
        for i in range(5):
            cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            ok, frame = cap.read()
            cap.release()
            if ok and frame.shape[1] / frame.shape[0] > 3:
                return i
        return None
    # Linux: lowest /dev/videoN named ZED (the capture node; the next one is metadata)
    nodes = sorted(Path("/sys/class/video4linux").glob("video*"), key=lambda p: int(p.name[5:]))
    for node in nodes:
        if "ZED" in (node / "name").read_text():
            return int(node.name[5:])
    return None


def open_zed(index=None, res="720"):
    """Open the ZED at a resolution. Returns (capture, index) or (None, None)."""
    width, height = RESOLUTIONS[res]
    index = index if index is not None else find_zed()
    if index is None:
        return None, None
    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2
    for _ in range(5):  # right after a USB reconnect the device can report "busy" briefly
        cap = cv2.VideoCapture(index, backend)
        if cap.isOpened():
            break
        time.sleep(1)
    else:
        return None, None
    if backend == cv2.CAP_V4L2:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap, index


def split_stereo(frame, upside_down=False):
    """(left, right) as the world sees them. Mounted upside down, each image is rotated 180° and
    the camera's right lens becomes the world's left."""
    half = frame.shape[1] // 2
    left, right = frame[:, :half], frame[:, half:]
    if upside_down:
        left, right = cv2.rotate(right, cv2.ROTATE_180), cv2.rotate(left, cv2.ROTATE_180)
    return left, right


def lens_intrinsics(conf_path, section="HD", side="LEFT"):
    """(K, dist) from a ZED factory calibration file for the RAW (unflipped) image of one lens."""
    cfg = configparser.ConfigParser()
    cfg.read(conf_path)
    s = cfg[f"{side}_CAM_{section}"]
    K = np.array([[float(s["fx"]), 0, float(s["cx"])], [0, float(s["fy"]), float(s["cy"])], [0, 0, 1]])
    dist = np.array([float(s[k]) for k in ("k1", "k2", "p1", "p2", "k3")])
    return K, dist


def lens_from_session(session):
    """(K, dist) of the raw left image of a lidarzed-record session (uses its copy of the .conf)."""
    session = Path(session)
    meta = json.loads((session / "meta.json").read_text())
    return lens_intrinsics(session / Path(meta["zed"]["calib_file"]).name, meta["zed"]["calib_section"])


def main():
    ap = argparse.ArgumentParser(description="Live ZED viewer (plain USB camera, no ZED SDK).")
    ap.add_argument("--index", type=int, help="camera index (auto-detected if omitted)")
    ap.add_argument("--res", default="720", choices=RESOLUTIONS)
    ap.add_argument("--upside-down", action="store_true", help="camera mounted upside down: rotate and swap")
    args = ap.parse_args()

    cap, index = open_zed(args.index, args.res)
    if cap is None:
        raise SystemExit("ZED not found. Check the USB cable/port and camera permission.")
    print(f"Opened camera {index} at {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame grab failed")
            break
        left, right = split_stereo(frame, args.upside_down)
        cv2.imshow("ZED left", left)
        cv2.imshow("ZED right", right)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            stamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(f"left_{stamp}.png", left)
            cv2.imwrite(f"right_{stamp}.png", right)
            print(f"Saved left_{stamp}.png / right_{stamp}.png")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
