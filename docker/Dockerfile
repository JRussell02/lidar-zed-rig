# direct_visual_lidar_calibration + a virtual X display, so `calibrate` runs headless over SSH
# (the tool opens an OpenGL viewer even with --background).
#   docker build -t lidarzed-vlcal:humble docker/
FROM koide3/direct_visual_lidar_calibration:humble
RUN apt-get update \
 && apt-get install -y --no-install-recommends xvfb xauth \
 && rm -rf /var/lib/apt/lists/*
