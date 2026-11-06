# Calibration 2026-11-05 (tripod-grip mount)

Camera ↔ LiDAR calibration for the tripod-grip version of the mount (`hardware/tripod_grip_mount.stl`): same plate, OS1 pattern and ZED mount as the grip, so the camera sits where it did on 2026-10-28; reassembly moved it by 0.6°. Valid only while nothing moves on the mount. OS1 on firmware 3.2.0.

| File | Contents |
|---|---|
| `T_lidar_camera.json` | **Calibration in use.** Camera pose in the OS1 sensor frame: t = (73.8, −48.0, −48.6) mm; 0.61° and 6 mm from 2026-10-28 |
| `init_T_lidar_camera.json` | Starting guess: the 2026-10-28 calibration |
| `per_recording/` | Each of `tri-calib01`–`05` calibrated alone (cross-check) |
| `camera_delay.json` | ZED capture delay **26 ms**, carried over (a camera/USB property, not the mount) |
| `SN17516.conf` | ZED factory lens calibration |

**Recordings.** `tri-calib01`–`05`: 60 s each, 19–29 s held still, then slow turns, tilts and a small step; two face the cabinet wall, two the window end, one the door end. `tri-height01`: 30 s still, box stack ~1.65 m away, wall ~6.3 m behind.

**Edge alignment** (`lidarzed-edge-score`, median px from LiDAR depth edges to image edges; lower is better):

| Recording | 2026-10-28 | This calibration |
|---|---:|---:|
| tri-calib01 | 4.0 | 3.0 |
| tri-calib02 | 2.8 | 2.0 |
| tri-calib03 | 4.0 | 3.0 |
| tri-calib04 | 4.0 | 2.8 |
| tri-calib05 | 3.0 | 2.0 |
| **Average** | **3.56** | **2.56** |

**Consistency.** Each recording calibrated alone lands within **0.03–0.15°** of the joint rotation. Positions spread more: 2–10 mm, mostly height (−5.7 to +10.1 mm) and sideways for `tri-calib04` (+9.5 mm; window and glass tank in view).

**Height.** `lidarzed-height` on `tri-height01` gives a clear minimum at **−51.5 mm** (near edges rise ~11 px by −10 mm), 3 mm from the joint result's −48.6 mm. Kept the joint value: 3 mm is under 2 px of color shift at 1 m. This also settles the 2026-10-28 question: the camera's optical center is ~50 mm below the OS1 base, not 24 mm.
