# Project Log: Building the OS1 + ZED Scanning Setup

How we got from "how do I connect this camera?" to a working, calibration-ready LiDAR + camera rig: what we worked on, what went wrong, and how we fixed it. Covers the work before the calibration session (see the [calibration report](calibration-report.md)); the grip itself is described in [hardware/](../hardware/).

**Period:** 22–28 Oct 2026 · **Hardware:** Ouster OS-1-128, Stereolabs ZED (1st gen), MacBook Pro (Apple Silicon), Linux mini PC

---

## Contents

1. [Connecting the ZED camera on a Mac](#1-connecting-the-zed-camera-on-a-mac)
2. [Planning the LiDAR + camera combination](#2-planning-the-lidar--camera-combination)
3. [Measuring scan fidelity](#3-measuring-scan-fidelity)
4. [Tooling: CloudCompare, Docker, handoff notes](#4-tooling-cloudcompare-docker-handoff-notes)
5. [Choosing a calibration approach](#5-choosing-a-calibration-approach)
6. [Moving to a shared Linux mini PC](#6-moving-to-a-shared-linux-mini-pc)
7. [Bringing both sensors up on the mini PC](#7-bringing-both-sensors-up-on-the-mini-pc)
8. [Building the recorder and converter](#8-building-the-recorder-and-converter)
9. [Challenges and solutions at a glance](#9-challenges-and-solutions-at-a-glance)
10. [Key decisions](#10-key-decisions)
11. [Lessons learned](#11-lessons-learned)

---

## 1. Connecting the ZED camera on a Mac

**Goal:** get the ZED working on a MacBook.

**Challenge: the ZED SDK does not run on macOS.** It requires an NVIDIA GPU with CUDA (Windows, Linux or Jetson only).

**Solution:** the USB ZED models also present themselves as a standard webcam (UVC). Each frame is the left and right images side by side, which OpenCV reads directly. We wrote a viewer (now `lidarzed-view`) that auto-detects the ZED, supports 2K / 1080 / 720 / VGA and saves left/right frames. The first capture worked at full 1280×720 per eye.

**Identifying the model.** The USB descriptor gave vendor `0x2b03` (Stereolabs), product `0xf582`, name "ZED": the **original 1st-generation ZED, revision B**. Later checks confirmed:

- **No IMU.** The camera exposes only two USB video interfaces and no HID (sensor) interface. Newer models (ZED Mini / 2 / 2i) add one.
- **Factory lens calibration** downloads from Stereolabs by serial number. A mistyped serial returned a generic placeholder file with round numbers and identical left/right values, which we spotted and replaced.

**Upside-down mounting.** The ZED hangs upside down on the grip. For viewing, the images are rotated 180° and left/right are swapped (the camera's right lens now sees the world's left). For coloring LiDAR, raw images are kept and the flip goes into the camera ↔ LiDAR calibration instead.

---

## 2. Planning the LiDAR + camera combination

**Roles:** the OS1 provides geometry (accurate range, 360° around); the ZED provides color and texture. The original ZED's own depth is weak beyond a few meters, so it is used as a color camera only.

**Pipeline we settled on:**
1. Record both sensors with timestamps on one clock.
2. Calibrate the camera's lens (factory file) and the camera ↔ LiDAR offset (extrinsics).
3. Run LiDAR SLAM for the rig's path, then color the points from the camera.

**Only one IMU.** The ZED has none, so the OS1's IMU is the only one. That simplifies things: no camera-IMU calibration is needed, because the camera's pose follows from LiDAR pose × fixed extrinsic.

**Timing without hardware sync.** The original ZED has no sync input, so both sensors are timestamped by the computer: LiDAR packets by arrival time (`ouster-cli save --ts packet`), camera frames by arrival time. The camera's constant delay is measured later (26 ms).

---

## 3. Measuring scan fidelity

To compare scans and find what improves them, we defined four measurements (now `lidarzed-eval`):

1. **Drift:** start-to-end gap of the SLAM trajectory (every scan starts and ends on a floor mark).
2. **Repeatability:** ICP-align repeated scans and report cloud-to-cloud distance (RMS).
3. **Absolute accuracy:** tape or laser-meter measurements vs. the same distances picked in CloudCompare.
4. **Sharpness:** plane-fit RMS of a flat wall patch (doubled walls from drift show up here).

The evaluator was tested on synthetic room scans: it recovered a known 15 cm / 3° offset, fixed a 95°-rotated scan with a heading search, and initially merged a misaligned scan. We fixed that so scans with low overlap are skipped instead of merged.

A test plan (repeat scans, a low pass under tables, slower walking, higher resolution, higher frame rate) was set up, but the team decided to go straight to the combined LiDAR + camera rig.

---

## 4. Tooling: CloudCompare, Docker, handoff notes

- **CloudCompare** for viewing and measuring point clouds. When a `.ply` "wouldn't open", the file was fine (8 M points read in 2 s); double-click was going to the wrong app. Fix: File → Open, or set CloudCompare as the default app for `.ply`.
- **Colima + Docker CLI** on the Mac for the Linux-only calibration tool. The VM was sized to 4 CPUs / 4 GB on an 8 GB Mac; the tool's x86 image runs through Rosetta.
- **Disk space:** the Mac had ~8 GB free, too little for a 5.4 GB calibration image plus recordings. Space was freed before pulling it.
- **Handoff notes:** a running notes file (status, decisions, open items) made it easy to switch accounts and pick the work back up later.

---

## 5. Choosing a calibration approach

**Question: "Do I need to print a checkerboard?"** No.

We chose the targetless toolbox **[direct_visual_lidar_calibration](https://github.com/koide3/direct_visual_lidar_calibration)** (Koide et al., ICRA 2023):

- It aligns the LiDAR intensity image with the camera image using the room's own structure (normalized information distance), so no target is needed.
- It handles spinning LiDARs by **merging a slow handheld sweep into a dense cloud**, which matters for a 128-beam sensor.
- A paper checkerboard wouldn't help anyway: at 2 m the OS1's beam rows are ~1.2 cm apart, and paper flat on a wall has no depth edge.
- Weakness: it needs a decent starting guess and a structured scene, and failures are less obvious, so every result gets an overlay check and a cross-check between recordings.

**Lens calibration** came from the ZED's factory file, so only the camera ↔ LiDAR extrinsics remained.

**Starting guess from CAD.** Instead of clicking matching points in the tool's GUI (impossible over SSH), we computed the guess from the grip's Blender model: camera ~78 mm forward, 60 mm to the side and 25 mm below the OS1 base, facing forward, upside down. Sanity-checked by projecting test points (forward → image center; up → down and left → right in the upside-down image).

---

## 6. Moving to a shared Linux mini PC

The recording and calibration moved to a **Linux mini PC** (Ubuntu 20.04, quad-core i5, 7.6 GB RAM, no NVIDIA GPU) with a built-in **gigabit Ethernet port** and USB 3. It gives the OS1 a gigabit connection without extra hardware and runs the x86 calibration image natively.

**Challenge: the machine is shared with another project's experiments.** We adopted a rule: our work must never delete, touch or change anything belonging to the other project. In practice:

- Everything lives in **one project folder**; Python 3.11 and the Ouster SDK were installed by `uv` *inside* that folder, leaving the system Python and shell config untouched (verified afterwards).
- Our containers have a **project prefix**, run with `--rm`, and only those are managed. No `docker prune`, Docker restarts, system upgrades or reboots.
- System changes (an Ethernet profile, camera permissions) were made only with approval and recorded.
- Heavy jobs run at the **lowest CPU priority** with a **memory cap**, so the other project's work always comes first.

**Can the OS1 use the Ethernet port?** Some lab equipment (for example network-attached radios) connects over Ethernet, so we checked first, read-only, and confirmed with the other team that the port was free. We created a separate profile that **never connects automatically**, so it can't interfere.

**Challenge: a side effect of our profile.** After the mini PC lost power and rebooted, Ubuntu's default "Wired connection 1" was **gone**. NetworkManager only creates that profile in memory when no other profile exists for the port, so ours suppressed it. We owned the mistake and **restored it as a saved profile** with the original settings.

**Other finding from that reboot:** the other project's services had stopped (no clean shutdown: likely a power loss) and are set not to restart. Per the rule, we didn't start them and flagged it to their owners.

---

## 7. Bringing both sensors up on the mini PC

**OS1:** an **OS-1-128**, firmware 3.1.0, mode 2048×10, vertical view +21.1° to −21.5°. Its metadata confirmed the measurement center is 38.2 mm above the base, which matches the CAD-based starting guess. With these numbers, the plate's front edge clears the lowest beam by ~13 mm.

| Challenge | Solution |
|---|---|
| **Accidental LiDAR unplug** made the sensor unreachable: Ubuntu's default profile grabbed the port when the link returned | `scripts/os1_up.sh`: re-activates the OS1 profile and waits for the sensor to boot |
| **Mini PC dropped off Wi-Fi** (and once powered off) | Physical check; reconnected over the same network |
| **The Mac switched Wi-Fi networks** mid-session, so SSH timed out | Rejoin the network the mini PC is on |
| **ZED unreadable over SSH:** `/dev/video*` belongs to the `video` group | With approval, added the user to `video` (camera access only) |
| **ZED "device busy"** right after it opened | The kernel log showed the ZED **reconnecting several times** (loose USB). Re-seated cables; the recorder now retries for a few seconds |
| **"Which way is forward?"** test with a person standing in front gave nonsense | Only 2% of the scene matched between captures: the rig had been **moved** between them. Skipped the test; it was confirmed later with real data (the CAD guess lines up; the 180°-flipped guess shows only the operator behind) |

On the mini PC the ZED delivered **2K at 15 fps and 1080/720/VGA at 30 fps** on USB 3.

---

## 8. Building the recorder and converter

### Recorder: `lidarzed-record`

Records the OS1 (through `ouster-cli`, raw OSF indexed by packet arrival time) and ZED frames (JPEGs with arrival timestamps in `frames.csv`) into one session folder.

| Challenge | Solution |
|---|---|
| Must work on the Mac (AVFoundation) and Linux (V4L2) | Backend chosen per OS; settings from a config file |
| A second Ctrl+C could abort ouster-cli mid-write | ouster-cli runs in its own process group and gets a single SIGINT from the recorder |
| `ouster-cli` always exits with **−6 ("terminate called…")** on stop | The file is fine; the recorder now **reads the OSF back** and reports how many scans are usable |
| ouster-cli's progress bar flooded the SSH output | Sent to `lidar.log` when there's no viewer |
| ouster-cli takes seconds to start, so "hold still" could end before LiDAR data existed | The recorder waits until LiDAR data is being written, then prints **HOLD STILL** / **MOVE SLOWLY** |
| A LiDAR failure left a camera-only session that looked successful | It stops the camera and prints a clear **LiDAR FAILED** message |
| No screen over SSH | The viewer is skipped automatically on Linux without a display |

### Converter: `lidarzed-make-bags`

The calibration tool reads ROS 2 bags; we have OSF + JPEG. Reading the tool's source showed what it needs:

- It uses **only the first image** in each bag and builds the dense cloud in the frame of the **first scan**, so the rig must be still at the start.
- Each point needs a **per-point time** (`t`) for motion correction.
- It accepts JPEGs directly as a "compressed" image topic.

We wrote the bags with the pure-Python `rosbags` library (no ROS install, no mixing the Ouster SDK into the tool's container). Verified details: the OSF keeps per-packet arrival times; the SDK's points are in the OS1's **sensor frame** (beam origins at z = 38.3 mm, matching the metadata); the first scan after start is incomplete and is skipped.

**Challenge:** the first conversion paired the reference image **788 ms** after the first scan, because ouster-cli started before the hold-still window. **Fix:** the reference scan is now the first one that starts *after* the window begins (test: −12 ms).

The tool's preprocessing read our bags on the first try (camera model and factory intrinsics detected, 4.78 M points merged from a 6 s test).

---

## 9. Challenges and solutions at a glance

| # | Challenge | Solution |
|---|---|---|
| 1 | ZED SDK not available on macOS | Use the ZED as a UVC webcam with OpenCV |
| 2 | Unsure which ZED model / whether it has an IMU | USB IDs and interfaces: original ZED rev B, no IMU |
| 3 | Camera mounted upside down | 180° flip + left/right swap for viewing; flip goes into the extrinsic for coloring |
| 4 | No way to judge scan quality | Four fidelity metrics + `lidarzed-eval` |
| 5 | `.ply` wouldn't open | File was fine; wrong default app |
| 6 | Low disk / 8 GB RAM on the Mac | Freed space; Docker VM sized to 4 GB; heavy work moved to the mini PC |
| 7 | Switching accounts loses context | A handoff notes file |
| 8 | Checkerboard or not? | Targetless toolbox with a CAD-based starting guess |
| 9 | Shared lab machine | Isolated folder, prefixed containers, approval for system changes, low priority |
| 10 | Ethernet port possibly needed by other equipment | Read-only checks + confirmation; non-autoconnecting profile |
| 11 | Our profile hid Ubuntu's default wired profile | Restored it as a saved profile |
| 12 | Another project's services stopped after a power loss | Left alone; flagged to their owners |
| 13 | LiDAR unplug breaks the network link | `scripts/os1_up.sh` |
| 14 | ZED not readable over SSH / "busy" | `video` group (approved); reseat USB; open retries |
| 15 | Forward-axis test failed | Rig moved between captures; confirmed later from real data |
| 16 | ouster-cli exit −6 and progress noise | Verify the OSF after each recording; log to file |
| 17 | Reference image paired 788 ms off | Pair inside the hold-still window (−12 ms) |

---

## 10. Key decisions

- Use the original ZED as a **color camera only**; no ZED SDK.
- The OS1's IMU is the **only IMU**; timestamps on the computer clock.
- **Record raw data every time** so settings can be retried without going back into the field.
- **Targetless calibration** with a **CAD-based starting guess**; no checkerboard.
- Run recording and calibration on a **Linux mini PC**, strictly isolated from the other project sharing it.
- Go straight to the combined LiDAR + camera rig.
- Run room scans from a **rolling cart** carrying the mini PC and a power bank.

---

## 11. Lessons learned

- **Read the tool's source when the docs are thin.** It told us the first image is used, the first scan defines the frame, `t` is required, and JPEGs are accepted.
- **On a shared machine, every system change has side effects.** Creating one Ethernet profile removed another; check and record before and after.
- **A failed test can be a setup problem.** The forward-axis test failed because the rig moved, not because the assumption was wrong.
- **Verify outputs automatically.** ouster-cli's crash-on-exit looked alarming; reading the file back settled it every time.
- **Keep handoff notes.** They made switching accounts and picking up later painless.
