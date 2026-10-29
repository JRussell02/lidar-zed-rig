"""Record the OS1 LiDAR and the ZED camera together, both timestamped on the computer's clock.

LiDAR goes through ouster-cli (raw OSF indexed by packet arrival time, optional live SLAM viewer).
ZED left images (raw, unflipped) are saved as JPEGs with arrival timestamps in frames.csv.

  lidarzed-record scan01                          # settings from config/rig.toml
  lidarzed-record calib01 --no-viz --duration 60  # calibration sequence: hold still, then move slowly
  lidarzed-record test --no-lidar --duration 5    # camera only

Output: <sessions>/NAME/{lidar.osf, lidar.log, zed/left/*.jpg, zed/frames.csv, meta.json, <lens .conf>}
Stop with Ctrl+C (also stops when the viewer window is closed or --duration elapses).
"""
import argparse
import csv
import json
import os
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
import tomllib
from pathlib import Path

import cv2

from .camera import CALIB_SECTION, RESOLUTIONS, open_zed

DEFAULT_CONFIG = Path("config/rig.toml")


def load_config(path):
    path = Path(path) if path else DEFAULT_CONFIG
    if not path.exists():
        return {}
    with open(path, "rb") as f:
        return tomllib.load(f)


def ouster_cli():
    """ouster-cli from the same environment as this package, else from PATH."""
    local = Path(sys.executable).with_name("ouster-cli")
    return str(local) if local.exists() else shutil.which("ouster-cli")


def writer(q, out_dir, jpeg_quality):
    """Save frames off the capture thread so slow disk writes don't drop frames."""
    params = [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]
    while True:
        item = q.get()
        if item is None:
            return
        name, left, right = item
        cv2.imwrite(str(out_dir / "left" / name), left, params)
        if right is not None:
            cv2.imwrite(str(out_dir / "right" / name), right, params)


def count_osf_frames(osf):
    """Read a finished OSF back (ouster-cli tends to exit with -6 on stop even when the file is fine)."""
    from .lidar import frames, open_osf
    try:
        src, _ = open_osf(osf)
        n = sum(1 for _ in frames(src, complete_only=False))
        src.close()
        return n
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", help="session name, e.g. calib01 or scan01")
    ap.add_argument("--config", help=f"rig config TOML [{DEFAULT_CONFIG}]")
    ap.add_argument("--sensor", help="OS1 hostname or IP (overrides config)")
    ap.add_argument("--sessions", help="folder for sessions (overrides config) [sessions]")
    ap.add_argument("--res", choices=RESOLUTIONS, help="ZED resolution [720]")
    ap.add_argument("--fps", type=float, help="max ZED frames saved per second [15]")
    ap.add_argument("--both", action="store_true", help="also save the right image")
    ap.add_argument("--quality", type=int, default=95, help="JPEG quality [95]")
    ap.add_argument("--index", type=int, help="ZED camera index (auto-detected if omitted)")
    ap.add_argument("--no-lidar", action="store_true", help="record the ZED only")
    ap.add_argument("--no-viz", action="store_true", help="record LiDAR without the live SLAM viewer")
    ap.add_argument("--slam-voxel", type=float, default=0.2, help="live SLAM voxel size (m) [0.2]")
    ap.add_argument("--duration", type=float, help="stop automatically after this many seconds")
    ap.add_argument("--hold", type=float, default=3, help="seconds to hold the rig still at the start [3]")
    args = ap.parse_args()

    cfg = load_config(args.config)
    sensor = args.sensor or os.environ.get("SENSOR") or cfg.get("sensor", {}).get("host")
    cam_cfg = cfg.get("camera", {})
    res = args.res or cam_cfg.get("resolution", "720")
    fps = args.fps or cam_cfg.get("fps", 15)
    upside_down = cam_cfg.get("upside_down", True)
    lens_conf = Path(cam_cfg.get("lens_calibration", "calibration/2026-10-28/SN17516.conf"))
    sessions = Path(args.sessions or cfg.get("paths", {}).get("sessions", "sessions"))

    session = sessions / args.name
    if session.exists():
        raise SystemExit(f"{session} already exists; choose another name")
    if not args.no_lidar and not sensor:
        raise SystemExit("No sensor: set [sensor] host in config/rig.toml, pass --sensor, or use --no-lidar")
    if not lens_conf.exists():
        raise SystemExit(f"Lens calibration not found: {lens_conf} (set [camera] lens_calibration)")
    if not args.no_viz and sys.platform != "darwin" and not os.environ.get("DISPLAY"):
        print("No display (SSH session): recording without the live viewer")
        args.no_viz = True

    # Open the ZED first so a camera problem doesn't leave a stray LiDAR recording
    w, h = RESOLUTIONS[res]
    cap, index = open_zed(args.index, res)
    if cap is None:
        raise SystemExit("ZED not found or busy (check the USB cable; on Linux the user needs the video group)")
    ok, frame = cap.read()
    if not ok:
        raise SystemExit("Could not read from the ZED")
    if frame.shape[1] != w or frame.shape[0] != h:
        raise SystemExit(f"ZED gave {frame.shape[1]}x{frame.shape[0]}, expected {w}x{h}")

    zed_dir = session / "zed"
    (zed_dir / "left").mkdir(parents=True)
    if args.both:
        (zed_dir / "right").mkdir()

    lidar, cmd = None, None
    if not args.no_lidar:
        cli = ouster_cli()
        if not cli:
            cap.release()
            raise SystemExit("ouster-cli not found (pip install ouster-sdk)")
        cmd = [cli, "source", sensor, "save", "--ts", "packet", str(session / "lidar.osf")]
        if not args.no_viz:
            cmd += ["slam", "-v", str(args.slam_voxel), "viz", "-m", "--accum-num", "20", "--map", "--map-ratio", "0.05"]
        # Own process group: Ctrl+C reaches only this script, which then sends ouster-cli a single
        # SIGINT so it can finalize the OSF (a second one could abort the write)
        lidar_log = open(session / "lidar.log", "w") if args.no_viz else None
        lidar = subprocess.Popen(cmd, start_new_session=True, stdout=lidar_log,
                                 stderr=subprocess.STDOUT if lidar_log else None)
        # Wait until LiDAR data is actually being written, so the hold-still period covers both sensors
        osf = session / "lidar.osf"
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and lidar.poll() is None:
            if osf.exists() and osf.stat().st_size > 2_000_000:
                break
            time.sleep(0.2)
        if lidar.poll() is not None or not osf.exists():
            cap.release()
            raise SystemExit(f"LiDAR recorder failed to start (exit code {lidar.poll()}); see {session / 'lidar.log'}")
        for _ in range(5):  # drop camera frames buffered while waiting
            cap.grab()

    meta = {
        "name": args.name, "host": os.uname().nodename, "hold_s": args.hold, "started_host_ns": time.time_ns(),
        "zed": {"index": index, "res": res, "width_per_eye": w // 2, "height": h,
                "calib_file": lens_conf.name, "calib_section": CALIB_SECTION[res],
                "upside_down": upside_down, "images_flipped": False,
                "saved": "left+right" if args.both else "left", "max_fps": fps},
        "lidar": {"sensor": sensor, "command": cmd, "timestamps": "host packet receive (save --ts packet)"} if lidar else None,
    }
    shutil.copy(lens_conf, session / lens_conf.name)

    q = queue.Queue(maxsize=120)
    t = threading.Thread(target=writer, args=(q, zed_dir, args.quality), daemon=True)
    t.start()

    stop = False

    def on_sigint(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, on_sigint)

    print(f"Recording session '{args.name}' -> {session}  (Ctrl+C to stop)")
    if args.hold > 0:
        print(f">>> HOLD THE RIG STILL for {args.hold:.0f} s")
    half = w // 2
    period = 1.0 / fps
    next_save = 0.0
    n_saved = n_dropped = 0
    t0 = time.monotonic()
    hold_announced = args.hold <= 0
    with open(zed_dir / "frames.csv", "w", newline="") as f:
        log = csv.writer(f)
        log.writerow(["frame", "host_ns", "file"])
        while not stop:
            ok, frame = cap.read()
            host_ns = time.time_ns()  # arrival time; the camera's constant delay is calibrated later
            if not ok:
                print("ZED frame grab failed; stopping")
                break
            now = time.monotonic()
            if lidar and lidar.poll() is not None:
                print("LiDAR recorder exited; stopping camera")
                break
            if args.duration and now - t0 >= args.duration:
                break
            if not hold_announced and now - t0 >= args.hold:
                print(">>> MOVE SLOWLY now")
                hold_announced = True
            if now < next_save:
                continue
            next_save = max(next_save + period, now)  # keep cadence without bursting after a stall
            name = f"{n_saved:06d}.jpg"
            right = frame[:, half:].copy() if args.both else None
            try:
                q.put_nowait((name, frame[:, :half].copy(), right))
            except queue.Full:
                n_dropped += 1
                continue
            log.writerow([n_saved, host_ns, name])
            n_saved += 1

    cap.release()
    q.put(None)
    t.join()

    if lidar and lidar.poll() is None:
        lidar.send_signal(signal.SIGINT)  # let ouster-cli finalize the OSF
        try:
            lidar.wait(timeout=30)
        except subprocess.TimeoutExpired:
            lidar.terminate()

    elapsed = time.monotonic() - t0
    meta.update(stopped_host_ns=time.time_ns(), zed_frames=n_saved, zed_dropped=n_dropped,
                lidar_exit_code=lidar.returncode if lidar else None)
    print(f"Saved {n_saved} ZED frames in {elapsed:.1f} s ({n_saved / max(elapsed, 1e-9):.1f} fps), "
          f"{n_dropped} dropped (disk too slow)")
    if lidar:
        osf = session / "lidar.osf"
        if osf.exists() and osf.stat().st_size > 0:
            n = count_osf_frames(osf)
            meta["lidar_frames_readable"] = n
            status = f"{n} LiDAR frames readable" if n else "WARNING: OSF could NOT be read back"
            print(f"LiDAR: {osf} ({osf.stat().st_size / 1e6:.0f} MB, ouster-cli exit {lidar.returncode}): {status}")
        else:
            print(f"WARNING: LiDAR FAILED (exit code {lidar.returncode}): no OSF recorded. "
                  "Session is camera-only; check lidar.log, then delete it and retry.")
    (session / "meta.json").write_text(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
