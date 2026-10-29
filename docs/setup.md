# Setup

## Software

```bash
git clone https://github.com/JRussell02/lidar-zed-rig.git && cd lidar-zed-rig
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[calibration,plot,dev]"
pytest                        # synthetic tests, no hardware needed
```

Python 3.11+. Tested on macOS (Apple Silicon) and Ubuntu 20.04 (Python 3.11 installed with `uv`, leaving the system Python alone). Useful extras: [CloudCompare](https://www.cloudcompare.org) for viewing results, Docker for the calibration tool (on macOS, Colima works; the tool's image is x86 and runs through Rosetta).

## Ouster OS1

- **Gigabit Ethernet is required:** at 2048×10 with 128 channels the OS1 sends ~250 Mbps. A 100 Mbps USB adapter will not work (and macOS may not even load a driver for older ones). Use a built-in port or a gigabit USB-C adapter (Realtek RTL8153 or ASIX AX88179 work without drivers on macOS and Linux).
- **Addressing:** with no DHCP server, the OS1 and the computer use link-local addresses (169.254.x.x). `ouster-cli discover` finds the sensor's hostname (`os-<serial>.local`).
- **Linux:** use a dedicated NetworkManager profile with `ipv4.method link-local` and `autoconnect no`, so it can't take over the port from other uses. After an unplug or power cut, bring it back with `scripts/os1_up.sh <sensor>`.

  ⚠ Creating a profile bound to an interface stops NetworkManager from auto-creating its default "Wired connection 1" (which lives only in memory, so it disappears at the next reboot). If others rely on the port for normal DHCP, save that default as a real profile too.
- **IMU deskew** in ouster-cli's SLAM needs OS1 firmware 3.2+; on 3.1 it falls back to constant-velocity deskew, which is fine at walking speed.

## Stereolabs ZED (1st generation)

- The ZED SDK needs an NVIDIA GPU, so we use the ZED as a plain USB camera (UVC): each frame has the left and right images side by side.
- Use a **USB 3** port and seat both cable ends firmly; a loose cable causes reconnects that leave the device briefly "busy".
- **Linux over SSH:** add the user to the `video` group for camera access (`sudo usermod -aG video $USER`, then log in again).
- **Factory calibration:** download `SN<serial>.conf` from `https://calib.stereolabs.com/?SN=<serial>` (the serial is on the sticker). A wrong serial can return a generic placeholder file with round numbers; check that fx/fy/cx/cy differ between the left and right lenses.
- **Upside-down mounting:** images are recorded raw; `lidarzed-view --upside-down` rotates them 180° and swaps left/right for viewing. For coloring, the flip is part of the calibration.

## Calibration tool (Docker)

```bash
docker build -t lidarzed-vlcal:humble docker/
```

This adds a virtual display (Xvfb) to [direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration), whose `calibrate` step needs one even in background mode. `docker/vlcal.sh` runs a step with the working folder (`work/` by default) mounted as `/data`, at low CPU priority with a memory cap (`VLCAL_CPUS`, `VLCAL_MEMORY`). Preprocessing all recordings at once can exceed 8 GB of RAM: process them one at a time at `--voxel_resolution 0.01`.

## Timing

The original ZED has no hardware sync. Both sensors are timestamped by the computer: LiDAR packets on arrival (`ouster-cli ... save --ts packet`), camera frames on arrival. The camera's image arrives ~26 ms after capture for this rig (`calibration/2026-10-28/camera_delay.json`); `lidarzed-delay` measures it for yours.
