# Example data

Recordings are 300–600 MB each, so they are not stored in git. Sample sessions will be published separately (e.g. as a GitHub Release or on Zenodo) and linked here.

## Session layout

`lidarzed-record NAME` writes `sessions/NAME/`:

```
sessions/calib05/
├── lidar.osf          raw OS1 recording (ouster-cli OSF), indexed by packet arrival time
├── lidar.log          ouster-cli output
├── zed/left/          000000.jpg, 000001.jpg, ... raw left images (upside down if the camera is)
├── zed/frames.csv     frame, host_ns (arrival time on the computer clock), file
├── meta.json          resolution, hold time, sensor, lens calibration section, frame counts
└── SN17516.conf       copy of the ZED factory lens calibration used
```

## Try it with your own recording

```bash
lidarzed-record test01 --duration 60
ouster-cli source sessions/test01/lidar.osf slam -v 0.1 save processed/test01_slam.osf
lidarzed-colorize sessions/test01 processed/test01_slam.osf --out processed/test01_color.ply
lidarzed-render processed/test01_color.ply --out test01.jpg
```
