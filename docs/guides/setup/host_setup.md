# Host Setup Notes

This repo now assumes a VIO-first host setup on the Jetson.

## What `setup_jetson.sh` Installs

- ROS 2 Humble via the ROS apt repository
- MAVROS and `mavros_extras`
- GeographicLib datasets for MAVROS
- `realsense2_camera`
- image transport / TF / IMU helper packages
- `colcon`, `vcstool`, `rosdep`
- minimal Python dependencies from `requirements-jetson.txt`
- RealSense udev rules from the bundled `librealsense` checkout when available

## Required Privileges

The script needs:

- sudo access for apt
- sudo access for `rosdep init`
- sudo access for RealSense udev rules
- sudo access for GeographicLib dataset install

## Commands

Run on the Jetson:

```bash
cd /home/jetson/cdrone_control
./setup_jetson.sh
```

Then:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
./scripts/check_vio_host_status.sh
ros2 launch drone_bringup realsense_d455.launch.py
./scripts/check_d455_topics.sh
```

## Current Known Blockers On This Machine

From the latest audit:

- ROS 2 Humble, MAVROS, and the active host packages are installed
- Docker access now works in this session
- the native host `realsense2_camera` path still fails on this Jetson even with the D455 back on USB 3
- the pinned Isaac ROS fallback now builds and detects the D455, but the camera still hits repeated low-level `control_transfer` / XU errors before usable streams were confirmed

The main blocker is no longer base package installation. It is the remaining D455 control / stream issue.
