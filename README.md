# cdrone_control

External-pose-first ROS 2 stack for getting a Jetson Orin Nano and an indoor pose source such as Intel RealSense D455 VSLAM or OptiTrack VRPN to feed trusted external vision into PX4 1.16 through MAVROS.

## Active Goal

This repo is no longer centered on the old target-tracking autonomy stack.

The active goal is:

1. bring up the active indoor pose source reliably on the Jetson or LAN
2. feed a trusted external pose estimate into MAVROS
3. get PX4 to trust that estimate indoors
4. validate `POSCTL` first, then restore `OFFBOARD`

The root problem is still the one captured in `problem.md`: PX4 is missing a trusted indoor aiding source, not a MAVROS link.

## Phase Status

Phase 0, Phase 1, and the first practical part of Phase 2 are now in place:

- the old autonomy-heavy workspace has been moved to `archive/legacy_stack/`
- the active ROS workspace now focuses on bringup, control bridging, and D455 sensor bringup
- `realsense_d455.launch.py` now targets a VIO-friendly D455 profile from a dedicated config file
- host bootstrap and validation scripts are now geared toward ROS 2 Humble + MAVROS + RealSense bringup
- the D455 is now working on the pinned Isaac ROS `release-3.2` RealSense stack
- Isaac ROS Visual SLAM is now publishing odometry from the live D455 path
- the repo now carries scripts to install VSLAM dependencies in-container, launch the RealSense-backed VSLAM stack, and capture evidence into a gitignored temp folder

What is still not done:

- the host-native `realsense2_camera` path is still not the recommended path on this Jetson
- a generic external-pose bridge now exists so RealSense VSLAM, OptiTrack VRPN, or any other `PoseStamped` source can feed the same MAVROS `vision_pose` path
- frame conversion, extrinsics, and PX4 fusion validation are still ahead of us

## Active Workspace

Active ROS packages under `ros2/src`:

- `drone_bringup`: MAVROS launch/config and D455 launch
- `drone_control_pkg`: keyboard teleop, generic external-pose adapters/bridges, and bench pose publisher
- `ros2_poselib`: leftover utility package kept for compatibility during the rehaul

Archived legacy packages:

- `archive/legacy_stack/ros2/src/drone_behavior_pkg`
- `archive/legacy_stack/ros2/src/drone_vision_pkg`
- `archive/legacy_stack/ros2/src/drone_light_pkg`
- `archive/legacy_stack/ros2/src/drone_monitor_pkg`

## Key Files

- `problem.md`: current PX4 indoor flight blocker
- `memory.md`: what the repo used to contain and what changed during the rehaul
- `rehaul_vio_px4_plan.md`: implementation plan
- `rehaul_vio_px4_viability.md`: external compatibility review
- `docs/rehaul/d455_vio_contract.md`: active D455 topic/rate/frame contract
- `docs/rehaul/host_setup.md`: host dependency setup notes
- `docs/rehaul/isaac_ros_release32.md`: pinned Isaac fallback and current container findings
- `docs/rehaul/isaac_vslam_d455.md`: active Visual SLAM bringup path and evidence workflow
- `problem2_realsense.md`: dated RealSense/VSLAM journal

## Active Launch Flows

Launch MAVROS only:

```bash
ros2 launch drone_bringup drone.launch.py
```

Launch the D455 with VIO-oriented defaults:

```bash
ros2 launch drone_bringup realsense_d455.launch.py
```

Run keyboard teleop for manual fallback:

```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

Use the legacy bench-only external-vision path if needed for plumbing checks:

```bash
ros2 launch drone_bringup bench_offboard.launch.py
```

Launch Isaac ROS Visual SLAM on the D455 path:

```bash
./scripts/start_isaac_realsense_container.sh
./scripts/launch_isaac_vslam_d455_in_container.sh
```

Bridge a generic external-pose source into MAVROS `vision_pose/pose`:

```bash
ros2 launch drone_bringup external_pose_px4_bridge.launch.py \
  pose_source:=optitrack \
  optitrack_server:=<motive-host>
```

Bridge the live RealSense VSLAM pose through the same contract:

```bash
ros2 launch drone_bringup vslam_px4_bridge.launch.py
```

Capture advisor-ready VSLAM evidence:

```bash
./scripts/capture_isaac_vslam_outputs.sh
```

## D455 Defaults

The active D455 launch is tuned for VIO-style input rather than RGB/depth demos:

- color disabled by default
- depth disabled by default
- infra1 and infra2 enabled
- gyro and accel enabled
- IMU unification enabled
- IR profile pinned to `640x360x90`
- gyro and accel pinned to `200 Hz`
- emitter disabled by default

The default profile lives in:

- `ros2/src/drone_bringup/config/realsense_d455_vio.yaml`

Validation script:

```bash
./scripts/check_d455_topics.sh
```

## Host Setup

The host bootstrap script now targets the active external-pose-first stack:

```bash
./setup_jetson.sh
```

It is designed to install:

- ROS 2 Humble
- MAVROS + mavros extras
- RealSense ROS driver
- image/TF/IMU support packages
- GeographicLib datasets
- minimal Python dependencies for the active repo

Current environment note:

- this Jetson is on Ubuntu `22.04.5`
- Jetson Linux is `R36.4.4`
- ROS 2 Humble is installed
- Docker access now works in this session
- the RealSense blocker is no longer the main issue
- the next blocker is validating the new VSLAM-to-MAVROS bridge with the flight controller online

Quick audit:

```bash
./scripts/check_vio_host_status.sh
```

## Archive

Legacy docs, launch files, and the old autonomy packages are preserved under:

```text
archive/legacy_stack/
```

Use that archive when you need to recover old behavior, test notes, or implementation details without letting the active repo drift back toward the old autonomy-first shape.
