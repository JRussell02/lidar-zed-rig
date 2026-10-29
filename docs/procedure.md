# Procedure: record, calibrate, color

Commands assume the package is installed (see [setup](setup.md)) and you're in the repository folder. Recordings go to `sessions/`, intermediate files to `work/` and `processed/` (all git-ignored).

## 1. Record

```bash
cp config/rig.example.toml config/rig.toml      # once: set your OS1 hostname
lidarzed-view --upside-down                     # optional: look through the camera
lidarzed-record scan01 --duration 240           # 4 min; stops by itself (or Ctrl+C)
```

A session folder holds `lidar.osf` (raw LiDAR, indexed by packet arrival time), `zed/left/*.jpg` (raw left images), `zed/frames.csv` (arrival timestamps), `meta.json` and a copy of the lens calibration.

**Scanning technique:** walk slowly (~0.5 m/s), turn slowly with your whole body, keep the rig level, and finish where you started. To see under tables, lower the rig slowly to ~0.5 m and walk the rows within the same recording.

## 2. Color a scan

```bash
ouster-cli source sessions/scan01/lidar.osf slam -v 0.1 save processed/scan01_slam.osf
lidarzed-colorize sessions/scan01 processed/scan01_slam.osf --out processed/scan01_color.ply
lidarzed-render processed/scan01_color.ply --out view.jpg      # quick look; or open the PLY in CloudCompare
```

SLAM stores a pose for every LiDAR column. `lidarzed-colorize` places each point in the world with its own column pose, picks the camera image captured when the beam swept the camera's view (arrival time − camera delay), projects the points into it with a depth test, and averages colors per 2 cm voxel. In CloudCompare, raise the point size and turn on the EDL shader for better depth perception.

## 3. Calibrate the camera to the LiDAR

Needed once per mount, and again whenever the camera or LiDAR moves on the grip.

### Record 3–5 calibration sequences

```bash
lidarzed-record calib01 --no-viz --duration 60 --hold 3
```

- **0–10 s:** hold completely still (the tool's reference image and scan come from here).
- **10–60 s:** slow turns (±15–20°), tilts (±10°), a gentle roll, a small step. Keep the scene in front of the camera.
- Face a different direction for each recording. Good scenes have structure and contrast **1–5 m** away (furniture, door frames, shelves); avoid blank walls, glass and bright windows. Keep people out of the camera's view.

### Process

```bash
docker build -t lidarzed-vlcal:humble docker/            # once
lidarzed-make-bags sessions/calib0[1-5] --out work/bags --max-seconds 60

# preprocess each recording separately (memory), 1 cm voxels
for b in calib01 calib02 calib03 calib04 calib05; do
  mkdir -p work/bags_one/$b && ln -sfn ../../bags/$b work/bags_one/$b/$b
  docker/vlcal.sh preprocess /data/bags_one/$b /data/pre_each/$b -d --voxel_resolution 0.01 \
    --image_topic /zed/left/image_raw/compressed --points_topic /os1/points \
    --camera_info_topic /zed/left/camera_info -i intensity
done
# combine: copy all per-recording outputs into work/pre and list every bag in its calib.json
mkdir -p work/pre && cp work/pre_each/*/calib0* work/pre/
python -c "import json; c=json.load(open('work/pre_each/calib01/calib.json')); c['meta']['bag_names']=['calib01','calib02','calib03','calib04','calib05']; json.dump(c, open('work/pre/calib.json','w'), indent=2)"

lidarzed-init-guess work/pre --init calibration/2026-10-28/init_T_lidar_camera.json
docker/vlcal.sh calibrate --data_path /data/pre --background --auto_quit
# result: results.T_lidar_camera in work/pre/calib.json
```

Store the result as a new dated folder in `calibration/` (`translation_m` + `quaternion_xyzw`).

### Check

```bash
lidarzed-overlay sessions/calib05 --T calibration/<date>/T_lidar_camera.json --out overlay.jpg --upright
lidarzed-edge-score sessions/calib0[1-5] --T calibration/<date>/init_T_lidar_camera.json calibration/<date>/T_lidar_camera.json
```

Calibrate each recording on its own too (point `calibrate` at `work/pre_each/<name>`): the rotations should agree within a few tenths of a degree. A recording that disagrees strongly usually had too little structure.

### Camera delay and height

```bash
lidarzed-delay sessions/calib01 sessions/calib02 sessions/calib03 sessions/calib05 \
    --T calibration/<date>/T_lidar_camera.json --min 0 --max 60 --step 2 --plot delay.png
lidarzed-height sessions/height01 --T calibration/<date>/T_lidar_camera.json --plot height.png
```

For the height test, record 30 s with the rig resting still, a box with a sharp horizontal top edge ~1 m in front of the camera and at least 1 m of open space behind it.

## 4. Scan fidelity

```bash
ouster-cli source sessions/A1/lidar.osf slam -v 0.1 save_trajectory processed/A1_traj.csv
lidarzed-eval drift processed/A1_traj.csv processed/A2_traj.csv      # start-to-end gap (start and end on a floor mark)
lidarzed-eval align processed/A1.ply processed/A2.ply --merge processed/merged.ply   # ICP repeatability + merge
lidarzed-eval refs measurements.csv                                  # tape vs scan distances
```

## Starting guess for a new mount

`calibrate` needs a rough starting pose. Compute it from your mount's CAD: the camera's position relative to the center of the OS1's base (+X away from the connector, +Z up), and its orientation for the **raw** image. Getting the 90°/180° orientation right matters; a few centimeters and degrees off is fine. See `calibration/2026-10-28/init_T_lidar_camera.json` for this grip's values.
