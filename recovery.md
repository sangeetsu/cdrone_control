# Recovery Guide: RealSense D455 VIO + Isaac ROS + PX4 Stack

This guide captures the current known-good recovery path for the RealSense D455
VIO stack on this Jetson so the environment can be rebuilt after a kernel
reflash or full rootfs wipe.

## Snapshot

Known-good snapshot date:

- 2026-03-25

Repo state:

- repo path: `/home/jetson/cdrone_control`
- git branch: `unstable_l2_realsense`
- git commit: `939318687e19e295872f70917122a09317fc8677`

Known-good Isaac bootstrap state:

- Isaac ROS common branch: `release-3.2`
- Isaac ROS common commit used here: `fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01`
- image key: `ros2_humble.realsense`

Known-good camera/runtime state:

- D455 firmware: `5.13.0.50`
- D455 USB link: `5000M`
- `realsense2_camera`: `4.51.1`
- `librealsense`: `2.55.1`
- Visual SLAM path verified from the D455 on 2026-03-24

Known-good Docker image:

- image name: `isaac_ros_dev-aarch64:latest`
- image id: `sha256:8f44d52096831bac8f89e70c94dfeac53020c06b003decbfe05c064893777b24`
- created: `2026-03-20T23:20:18.816654179Z`

Important note:

- the base image already contains the pinned RealSense stack
- the Isaac VSLAM apt packages are installed into the running container by
  `./scripts/ensure_isaac_vslam_deps_in_container.sh`
- because the runtime container is launched with `--rm`, those VSLAM packages
  are not the same thing as a permanently baked custom image

## What Must Be True For Recovery To Work

- The Jetson must be on Ubuntu 22.04 / ROS 2 Humble for the host-side scripts.
- Docker and the NVIDIA container runtime must be working.
- The D455 must still be on firmware `5.13.0.50`.
- The repo must be present at `/home/jetson/cdrone_control` or an equivalent
  path where the scripts can be run.

If the camera firmware drifts, use:

- [d455_firmware_downgrade.md](/home/jetson/cdrone_control/docs/rehaul/d455_firmware_downgrade.md)

## Backup Artifact

Primary export directory on this Jetson:

- `/home/jetson/recovery_exports`

Expected Docker export file:

- `/home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst`
- size: `13641422664` bytes
- sha256: `d844d2e10c85b4d1ce35bcfb69b35b5dc344607a34827c74e4b00913c618e929`

Expected checksum file:

- `/home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256`

Additional recovery artifacts:

- `/home/jetson/recovery_exports/cdrone_control_20260325.bundle`
- `/home/jetson/recovery_exports/recovery_manifest_20260325.txt`
- `/home/jetson/recovery_exports/vio_20260324T033034Z.tar.gz`

## Copy Options

### Option 1: Pull From Another PC Using SSH

This Jetson is listening on SSH port `22` and currently has IP:

- `192.168.0.155`

From another Linux or macOS machine on the same network:

```bash
scp jetson@192.168.0.155:/home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst .
scp jetson@192.168.0.155:/home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256 .
scp jetson@192.168.0.155:/home/jetson/recovery_exports/cdrone_control_20260325.bundle .
scp jetson@192.168.0.155:/home/jetson/recovery_exports/recovery_manifest_20260325.txt .
```

Then verify:

```bash
sha256sum -c isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256
```

### Option 2: Copy To A Mounted Storage Device

If a USB or external SSD is mounted, copy the files directly:

```bash
cp /home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst /path/to/mount/
cp /home/jetson/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256 /path/to/mount/
sync
```

### Option 3: Serve The Export Directory Over HTTP

From the Jetson:

```bash
python3 -m http.server 8000 --bind 0.0.0.0 --directory /home/jetson/recovery_exports
```

From another machine on the same network:

```bash
curl -O http://192.168.0.155:8000/isaac_ros_dev-aarch64_latest_20260325.tar.zst
curl -O http://192.168.0.155:8000/isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256
```

Then verify:

```bash
sha256sum -c isaac_ros_dev-aarch64_latest_20260325.tar.zst.sha256
```

### Option 4: Rebuild Instead Of Restore

If the export is unavailable, the stack can still be rebuilt from scripts and
internet access. That path is documented below.

## Fast Recovery Path After A Full Wipe

Use this path if the reflash wipes Docker images and the rootfs.

### 1. Restore The Repo

Clone the repo back onto the Jetson and return it to the known-good commit:

```bash
cd /home/jetson
git clone <your-repo-remote> cdrone_control
cd /home/jetson/cdrone_control
git checkout 939318687e19e295872f70917122a09317fc8677
```

If you keep a local mirror or backup tarball of the repo, restoring from that
is fine too.

If you want to recover the repo from the Git bundle instead of a remote:

```bash
cd /home/jetson
git clone /path/to/cdrone_control_20260325.bundle cdrone_control
cd /home/jetson/cdrone_control
git checkout 939318687e19e295872f70917122a09317fc8677
```

### 2. Restore Host ROS Packages And Workspace

Run:

```bash
cd /home/jetson/cdrone_control
./setup_jetson.sh
```

This restores the host ROS, MAVROS, RealSense host packages, GeographicLib
datasets, udev rules, Python requirements, and the local workspace build.

### 3. Verify Docker And NVIDIA Runtime

This repo does not install Docker for you. If a fresh reflash removed it, make
sure these are working before continuing:

```bash
docker version
docker info
```

Current known-good package family on this machine was:

- `docker-ce`
- `docker-ce-cli`
- `containerd.io`
- `nvidia-container`
- `nvidia-container-toolkit`

The user must also be able to access the Docker socket without sudo.

### 4. Confirm The D455 Firmware

The VIO path that worked here depended on the D455 being on:

- `5.13.0.50`

If the camera is not on that firmware, follow:

- [d455_firmware_downgrade.md](/home/jetson/cdrone_control/docs/rehaul/d455_firmware_downgrade.md)

### 5. Restore Or Rebuild The Isaac Docker Image

Preferred path if the backup exists:

```bash
zstd -dc /path/to/isaac_ros_dev-aarch64_latest_20260325.tar.zst | docker load
docker image ls isaac_ros_dev-aarch64:latest
```

Fallback if the backup does not exist:

```bash
cd /home/jetson/cdrone_control
./scripts/setup_isaac_ros_release32.sh
```

Then, for a closer match to the known-good environment, pin the external clone:

```bash
git -C /home/jetson/cdrone_control/external/isaac_ros_release32/src/isaac_ros_common checkout fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01
```

Then build the image from an interactive terminal:

```bash
cd /home/jetson/cdrone_control
./scripts/run_isaac_realsense_dev.sh
```

This build is large and will pull or rebuild a substantial Isaac image.

### 6. Start The Headless Isaac Container

```bash
cd /home/jetson/cdrone_control
./scripts/start_isaac_realsense_container.sh
```

### 7. Reinstall The Isaac VSLAM Packages Into The Running Container

This step matters on a fresh container because the working VSLAM path was not
captured as a separate committed image:

```bash
cd /home/jetson/cdrone_control
./scripts/ensure_isaac_vslam_deps_in_container.sh
```

### 8. Launch The RealSense-Backed VSLAM Stack

```bash
cd /home/jetson/cdrone_control
./scripts/launch_isaac_vslam_d455_in_container.sh
```

### 9. Start The Host-Side PX4 Bridge

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup vslam_px4_bridge.launch.py
```

### 10. Validate

Check the camera and VSLAM outputs:

```bash
cd /home/jetson/cdrone_control
./scripts/capture_isaac_vslam_outputs.sh
```

The expected healthy signals are:

- RealSense D455 visible on USB 3 / `5000M`
- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/camera/imu`
- `/visual_slam/status`
- `/visual_slam/tracking/odometry`
- `/visual_slam/tracking/vo_pose`
- `/mavros/vision_pose/pose`

The March 24 evidence bundle is here for comparison:

- [vio_20260324T033034Z](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z)

## Short Recovery Path If The Rootfs Survives

If the reflash only updates boot firmware, kernel, or device tree and keeps the
existing rootfs and Docker cache intact:

1. Verify the repo is still present.
2. Verify `docker image ls isaac_ros_dev-aarch64:latest` still shows the image.
3. Verify the D455 is still on `5.13.0.50`.
4. Run `./scripts/start_isaac_realsense_container.sh`.
5. Run `./scripts/ensure_isaac_vslam_deps_in_container.sh`.
6. Run `./scripts/launch_isaac_vslam_d455_in_container.sh`.
7. Launch `vslam_px4_bridge.launch.py`.

That is the lowest-friction path if the flash does not wipe `/`.

## Known Quirks

- The D455 IR stream can initially come up near `15 Hz` instead of `90 Hz`.
  The repo launch helpers automatically re-apply
  `depth_module.enable_auto_exposure=true` to recover the expected rate.
- The host-native RealSense path is not the preferred path on this Jetson for
  VIO. The validated path is the pinned Isaac container flow.
- `./scripts/setup_isaac_ros_release32.sh` follows the `release-3.2` branch by
  default. For a closer match to the known-good setup, re-checkout commit
  `fcf4d9e17f8f0a7f47f1d22d6a18421ce3768c01`.
- `./scripts/ensure_isaac_vslam_deps_in_container.sh` installs unpinned apt
  packages. That means the workflow is reproducible, but exact package versions
  can drift over time.

## Key Files

- [problem2_realsense.md](/home/jetson/cdrone_control/problem2_realsense.md)
- [isaac_ros_release32.md](/home/jetson/cdrone_control/docs/rehaul/isaac_ros_release32.md)
- [isaac_vslam_d455.md](/home/jetson/cdrone_control/docs/rehaul/isaac_vslam_d455.md)
- [d455_firmware_downgrade.md](/home/jetson/cdrone_control/docs/rehaul/d455_firmware_downgrade.md)
- [setup_jetson.sh](/home/jetson/cdrone_control/setup_jetson.sh)
- [setup_isaac_ros_release32.sh](/home/jetson/cdrone_control/scripts/setup_isaac_ros_release32.sh)
- [start_isaac_realsense_container.sh](/home/jetson/cdrone_control/scripts/start_isaac_realsense_container.sh)
- [ensure_isaac_vslam_deps_in_container.sh](/home/jetson/cdrone_control/scripts/ensure_isaac_vslam_deps_in_container.sh)
- [launch_isaac_vslam_d455_in_container.sh](/home/jetson/cdrone_control/scripts/launch_isaac_vslam_d455_in_container.sh)
- [capture_isaac_vslam_outputs.sh](/home/jetson/cdrone_control/scripts/capture_isaac_vslam_outputs.sh)
