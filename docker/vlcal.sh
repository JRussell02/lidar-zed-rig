#!/bin/bash
# Run a direct_visual_lidar_calibration step in a throwaway container. The working folder (bags,
# preprocessed data) is mounted as /data: $VLCAL_DATA, default ./work. Output files are handed back
# to the calling user (the container runs as root). Low CPU priority and a memory cap keep it polite
# on a shared machine; adjust with VLCAL_CPUS / VLCAL_MEMORY.
#   docker/vlcal.sh preprocess /data/bags_one/calib01 /data/pre_each/calib01 -d --voxel_resolution 0.01 \
#       --image_topic /zed/left/image_raw/compressed --points_topic /os1/points \
#       --camera_info_topic /zed/left/camera_info -i intensity
#   docker/vlcal.sh calibrate --data_path /data/pre --background --auto_quit
set -euo pipefail
step="$1"; shift
DATA="$(cd "${VLCAL_DATA:-work}" && pwd)"
exec docker run --rm --name "lidarzed-vlcal-${step}" --cpus "${VLCAL_CPUS:-2}" --cpu-shares 128 \
  --memory "${VLCAL_MEMORY:-6g}" -v "$DATA:/data" lidarzed-vlcal:humble \
  bash -c "source /root/ros2_ws/install/setup.bash && nice -n 19 xvfb-run -a -s '-screen 0 1920x1080x24' ros2 run direct_visual_lidar_calibration ${step} $*; rc=\$?; chown -R $(id -u):$(id -g) /data; exit \$rc"
