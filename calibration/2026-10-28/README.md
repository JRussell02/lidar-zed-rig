# Calibration 2026-10-28

Camera ↔ LiDAR calibration for the grip in `hardware/` (ZED mounted upside down under the front of the plate, OS1 on top, connector facing back). Valid only while nothing moves on the mount. Full write-up: [docs/calibration-report.md](../../docs/calibration-report.md).

| File | Contents |
|---|---|
| `T_lidar_camera.json` | **Calibration in use.** Camera pose in the OS1 sensor frame: t = (76.9, −52.2, −50.7) mm, ~1.2° from the CAD guess |
| `init_T_lidar_camera.json` | Starting guess from the CAD model: t = (78, −60, −25) mm, facing forward, upside down |
| `camera_delay.json` | ZED capture delay **26 ms** (image arrival − capture), ±3 ms across four recordings |
| `SN17516.conf` | ZED factory lens calibration (download yours from `calib.stereolabs.com/?SN=<serial>`) |
| `alternatives/` | Ruler-height variant (rotation refit) and a hybrid that scored worse |

**Conventions.** Transforms are `translation_m` + `quaternion_xyzw`: the pose of the camera that took the **raw (unflipped) left image** in the Ouster **sensor** frame (+X away from the connector, +Z up, origin at the center of the base). Use `LEFT_CAM_HD` from the `.conf` for 720p images.

**Accuracy.** Rotation agrees within ~0.2° across recordings. The camera's height is uncertain by 2–3 cm (data says ~50 mm below the OS1 base, a ruler says 24 mm); that is under ~4 px of color shift beyond 2 m.
