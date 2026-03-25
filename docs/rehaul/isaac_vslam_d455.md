# Isaac VSLAM D455 Bringup

This document records the active Isaac ROS Visual SLAM path for the repo's D455 on the Jetson Orin Nano.

## Current Status

As of 2026-03-24, the repo has moved past "camera bringup only" and into a working Visual SLAM stage.

Confirmed on this machine:

- D455 firmware: `5.13.0.50`
- USB link: `5000M`
- Isaac ROS branch: `release-3.2`
- Isaac image key: `ros2_humble.realsense`
- `realsense2_camera` in container: `4.51.1`
- `librealsense`: `2.55.1`
- `isaac_ros_visual_slam`: installed in the container and publishing odometry

## Repo Scripts

Install the Visual SLAM dependencies in the running Isaac container:

```bash
./scripts/ensure_isaac_vslam_deps_in_container.sh
```

Launch the RealSense-backed Visual SLAM pipeline:

```bash
./scripts/launch_isaac_vslam_d455_in_container.sh
```

Capture advisor-ready evidence to a gitignored temp folder:

```bash
./scripts/capture_isaac_vslam_outputs.sh
```

Launch the repo's first-pass PX4 bootstrap bridge on the host:

```bash
ros2 launch drone_bringup vslam_px4_bridge.launch.py
```

## What The Launch Script Does

The launch script uses the installed NVIDIA RealSense Visual SLAM launch:

```text
/opt/ros/humble/share/isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py
```

That launch:

- starts `realsense2_camera`
- starts `isaac_ros_visual_slam`
- enables IMU fusion
- remaps the RealSense stereo and IMU topics into the VSLAM node

This repo adds one Jetson-specific fix on top:

- after the RealSense node comes up, the script re-applies `depth_module.enable_auto_exposure=true`
- then applies it a second time after a short delay

That is needed because NVIDIA documents a D455-on-Jetson issue where the infrared stream can otherwise start capped near `15 Hz`.

## Expected Topics

Camera topics:

- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/camera/imu`

Visual SLAM topics:

- `/visual_slam/status`
- `/visual_slam/tracking/odometry`
- `/visual_slam/tracking/vo_pose`
- `/visual_slam/tracking/vo_pose_covariance`
- `/visual_slam/tracking/vo_path`
- `/visual_slam/tracking/slam_path`

## Observed Runtime Result On 2026-03-24

After the firmware downgrade and the D455 auto-exposure workaround:

- `/camera/infra1/image_rect_raw` measured about `89.9 Hz`
- `/camera/imu` was present
- `/visual_slam/tracking/odometry` measured about `89.9 Hz`
- `/visual_slam/status` published `vo_state: 1`
- `/visual_slam/tracking/odometry` produced a valid `nav_msgs/msg/Odometry` sample in the `odom -> camera_link` frame chain
- the repo bootstrap bridge relayed `/visual_slam/tracking/vo_pose` into `/mavros/vision_pose/pose`
- `/mavros/vision_pose/pose` measured about `30.0 Hz`
- `/mavros/companion_process/status` published `state: 4` for the visual-inertial odometry component

One captured odometry sample and the rate logs are stored in the gitignored temp output directory created by the capture script.

## Evidence Folder

The capture script writes into:

```text
temp_outputs/vio_<timestamp>/
```

Typical files include:

- `summary.txt`
- `realsense_device.txt`
- `ros_nodes.txt`
- `ros_topics.txt`
- `visual_slam_status_once.txt`
- `visual_slam_odometry_once.txt`
- `visual_slam_odometry_hz.txt`
- `camera_infra1_hz.txt`
- `camera_imu_hz.txt`
- `mavros_vision_pose_once.txt`
- `mavros_vision_pose_hz.txt`
- `mavros_companion_status_once.txt`
- `isaac_vslam_launch_tail.txt`
- `isaac_vslam_post_fix.txt`

## What This Means For The Rehaul

The next rehaul step is no longer "make the D455 stream."

The repo now has the first bootstrap bridge from Isaac VSLAM into MAVROS
`vision_pose/pose`, so the next rehaul step is:

1. keep this Isaac Visual SLAM path stable
2. validate the new pose-bridge path with the FCU online
3. replace the bootstrap pose relay with the final frame-aware odometry bridge
4. validate PX4 fusion in bench tests before flight tests

## Remaining Gaps

What is still missing before indoor PX4 position-hold:

- FCU-online validation of the new bootstrap pose bridge
- fixed camera-to-body extrinsics for this airframe
- explicit ENU/NED and timestamp handling for PX4
- the final odometry-based bridge path
- PX4 external-vision fusion parameter validation
- bench validation with the flight controller online

So VSLAM is now alive, but PX4 integration is still the next real engineering task.
