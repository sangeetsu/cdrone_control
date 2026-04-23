# Problem 2 Journal: RealSense D455 And VSLAM Bringup

This file is now a dated journal for the RealSense and VSLAM investigation so we can see what changed on each session date without reconstructing the story from memory.

## Current State

As of 2026-03-24:

- the D455 is on firmware `5.13.0.50`
- the camera negotiates on USB 3 / `5000M`
- the active Isaac path matches the pinned compatibility set:
  - Isaac ROS `release-3.2`
  - `CONFIG_IMAGE_KEY=ros2_humble.realsense`
  - `realsense2_camera 4.51.1`
  - `librealsense 2.55.1`
- the D455 stream is working on the Isaac container path
- Isaac ROS Visual SLAM is publishing live odometry from the D455
- the new remaining blocker is PX4 integration, not camera bringup

## 2026-03-24

### Goal

Verify the colleague's firmware downgrade, confirm stack compatibility, prove the D455 path is healthy, and move into the next rehaul phase by standing up live Visual SLAM.

### What Was Verified

- `lsusb -t` showed the D455 on the SuperSpeed path at `5000M`
- `rs-enumerate-devices` inside the Isaac container reported:
  - `Intel RealSense D455`
  - serial `135222250343`
  - firmware `5.13.0.50`
  - USB type `3.2`
- the Isaac container remained on the intended RealSense stack:
  - `realsense2_camera 4.51.1`
  - `librealsense2.so.2.55.1`
- this matched NVIDIA's documented `release-3.2` guardrail for RealSense

### What Changed Compared To The Earlier Failure

After the firmware downgrade:

- live `/d455/gyro/sample` data flowed again at about `200 Hz`
- live `/d455/imu` data flowed
- live `/d455/infra1/image_rect_raw` frames flowed
- the earlier `get_xu(ctrl=1)` hard failure did not return in the same way

There was still one known Jetson quirk:

- the D455 IR topic could start near `15 Hz` instead of `90 Hz`
- NVIDIA documents this exact issue on Jetson and recommends re-applying:

```bash
ros2 param set /camera/camera depth_module.enable_auto_exposure true
```

On this machine that workaround restored the IR stream to about `89.9 Hz`.

### Repo Changes Made On This Date

The repo was advanced from "camera recovery" into "working VSLAM":

- `scripts/launch_isaac_realsense_d455_in_container.sh`
  - kept and improved as the Isaac RealSense helper
- `scripts/ensure_isaac_vslam_deps_in_container.sh`
  - added to install VSLAM-related Isaac packages into the running container
- `scripts/launch_isaac_vslam_d455_in_container.sh`
  - added to launch the RealSense-backed Isaac ROS Visual SLAM stack
- `scripts/capture_isaac_vslam_outputs.sh`
  - added to save advisor-ready evidence into a gitignored temp folder
- `ros2/src/drone_bringup/launch/vslam_px4_bridge.launch.py`
  - added as the first-pass bridge from Isaac VSLAM `vo_pose` into MAVROS `vision_pose/pose`
- `.gitignore`
  - updated to ignore `/temp_outputs/`
- `docs/guides/vio/isaac_vslam_d455.md`
  - added as the active VSLAM bringup note
- `README.md`
  - updated to reflect that VSLAM is now alive
- `memory.md`
  - updated with the VSLAM milestone

### VSLAM Installation Work

Inside the running Isaac container, the following packages were installed:

- `ros-humble-isaac-ros-visual-slam`
- `ros-humble-isaac-ros-examples`
- `ros-humble-isaac-ros-realsense`

This was done from the NVIDIA Isaac apt repository already present in the container.

### VSLAM Runtime Result

The following launch was brought up successfully:

```bash
./scripts/launch_isaac_vslam_d455_in_container.sh
```

Observed nodes:

- `/camera/camera`
- `/visual_slam_node`
- `/visual_slam_launch_container`

Observed topic family:

- `/camera/infra1/image_rect_raw`
- `/camera/infra2/image_rect_raw`
- `/camera/imu`
- `/visual_slam/status`
- `/visual_slam/tracking/odometry`
- `/visual_slam/tracking/vo_pose`
- `/visual_slam/tracking/vo_pose_covariance`
- `/visual_slam/tracking/vo_path`
- `/visual_slam/tracking/slam_path`

Observed sample outputs:

- `/visual_slam/status` published `vo_state: 1`
- `/visual_slam/tracking/odometry` produced a valid `nav_msgs/msg/Odometry` message with:
  - `frame_id: odom`
  - `child_frame_id: camera_link`
- `/visual_slam/tracking/odometry` measured about `89.9 Hz`
- `/camera/infra1/image_rect_raw` measured about `89.9 Hz` after the post-launch auto-exposure workaround
- `/camera/gyro/sample` stayed near `200 Hz`
- the new host-side bootstrap bridge also relayed `/visual_slam/tracking/vo_pose` into `/mavros/vision_pose/pose`
- `/mavros/vision_pose/pose` measured about `30.0 Hz`
- `/mavros/companion_process/status` published `state: 4` and `component: 197`

### Evidence Captured

The complete evidence bundle for this date was written to:

- [temp_outputs/vio_20260324T033034Z](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z)

Important files in that folder:

- [summary.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/summary.txt)
- [realsense_device.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/realsense_device.txt)
- [visual_slam_status_once.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/visual_slam_status_once.txt)
- [visual_slam_odometry_once.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/visual_slam_odometry_once.txt)
- [visual_slam_odometry_hz.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/visual_slam_odometry_hz.txt)
- [camera_infra1_hz.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/camera_infra1_hz.txt)
- [camera_imu_hz.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/camera_imu_hz.txt)
- [mavros_vision_pose_once.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/mavros_vision_pose_once.txt)
- [mavros_vision_pose_hz.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/mavros_vision_pose_hz.txt)
- [mavros_companion_status_once.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/mavros_companion_status_once.txt)
- [isaac_vslam_launch_tail.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/isaac_vslam_launch_tail.txt)
- [isaac_vslam_post_fix.txt](/home/jetson/cdrone_control/temp_outputs/vio_20260324T033034Z/isaac_vslam_post_fix.txt)

### What This Means

By the end of this session:

- the RealSense camera path was no longer the main blocker
- the VIO/VSLAM engine was alive and producing live odometry
- the repo gained a bootstrap VSLAM-to-MAVROS pose bridge
- the rehaul moved into the next real problem: FCU-side PX4 integration and validation

### What Is Still Missing

The system is still not flight-ready for PX4 indoor position hold.

We still need:

1. FCU-online validation of the new bootstrap pose bridge
2. explicit camera-to-body extrinsics for this airframe
3. controlled frame conversion and timestamp handling
4. the final odometry-based bridge path
5. PX4 external-vision fusion parameter validation
6. FCU bench tests with the real VSLAM feed
7. only after that, indoor `POSCTL` validation and later `OFFBOARD`

### Next Rehaul Step

The next rehaul step is to keep using the Isaac VSLAM path and validate the new MAVROS bootstrap bridge against the flight controller.

That means:

- bench-test `/visual_slam/tracking/vo_pose` -> `/mavros/vision_pose/pose`
- confirm PX4 accepts and fuses the bridge output
- then replace the bootstrap pose relay with the final frame-aware MAVROS odometry path
- bench-test PX4 fusion before touching flight control logic

## 2026-03-21

### Goal

Retest the D455 after a full Jetson reboot and determine whether the earlier container failure was stale state or a deeper compatibility problem.

### What Happened

- the D455 came back on USB 3 / `5000M`
- the Isaac container still saw the camera
- the RealSense node still reached startup
- the same low-level control-transfer problems reproduced
- live IR and IMU samples still did not flow

### Deeper Checks

The investigation went below ROS:

- host kernel logs showed repeated `uvcvideo ... Unknown video format ...`
- host kernel logs showed repeated `Failed to query (GET_CUR) UVC control ...: -71`
- raw `v4l2-ctl` stream attempts produced zero-byte output files

### Conclusion

At the end of that date, the likely blocker was still the firmware/version mismatch and low-level device-control failure, so the next step became the D455 downgrade to `5.13.0.50`.

## 2026-03-20

### Goal

Complete the repo-side rehaul and get the host ready for the VIO-first stack.

### What Happened

- legacy autonomy-heavy packages were archived under `archive/legacy_stack/`
- the README and repo memory were rewritten around D455 VIO
- host setup and validation scripts were added
- the pinned Isaac ROS `release-3.2` fallback path was added
- the D455 was first diagnosed on a bad USB 2 cable before being moved back to USB 3
- the host-native RealSense path still failed with `bad optional access`
- the first Isaac-container tests saw the camera but did not yet produce live data

### Conclusion

The repo was narrowed to the right problem, but the camera stack was still not healthy enough to move on to VIO at the end of that session.

## Sources

- Intel RealSense D400 firmware releases:
  https://dev.realsenseai.com/docs/firmware-releases-d400
- Intel RealSense firmware update tool:
  https://dev.realsenseai.com/docs/firmware-update-tool
- Intel RealSense firmware/software update overview:
  https://dev.realsenseai.com/docs/firmware-updates
- NVIDIA Isaac ROS RealSense setup:
  https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/sensors/realsense_setup.html
- NVIDIA Isaac ROS Visual SLAM docs:
  https://nvidia-isaac-ros.github.io/v/release-3.2/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html
- NVIDIA Isaac ROS hardware troubleshooting:
  https://nvidia-isaac-ros.github.io/v/release-3.2/troubleshooting/hardware_setup.html
- Working Jetson reference:
  https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417
