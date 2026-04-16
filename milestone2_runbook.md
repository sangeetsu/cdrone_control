# Milestone 2 Runbook

This runbook is for the tracking-first integration that now lives inside
`cdrone_control`.

Current architecture:

- `drone_vision_pkg` owns the active perception node.
- `drone_bringup` owns the feature launch entry points.
- `cdrone_yolo` is currently treated as the model and algorithm source, not the
  runtime owner.
- `tracking_only` does not require the Isaac container.
- `tracking_only` uses direct `pyrealsense2` capture by default, so it does not
  depend on `realsense2_camera` for the bench milestone.
- `vslam_only` and `combined` launch entry points exist as scaffolding only.

## What Was Added

- `ros2/src/drone_vision_pkg`
- `drone_bringup/launch/tracking_only.launch.py`
- `drone_bringup/launch/vslam_only.launch.py`
- `drone_bringup/launch/combined.launch.py`
- `drone_bringup/config/realsense_d455_tracking.yaml`
- `drone_msgs/msg/WorldTargetTrack.msg`
- `drone_msgs/msg/WorldTargetTrackArray.msg`
- `scripts/check_realsense_host_access.sh`
- `scripts/install_realsense_host_access.sh`

Primary outputs from the tracker:

- `/cdrone/drone01/perception/tracks`
  Type: `drone_msgs/TargetTrackArray`
  Meaning: body-frame target positions relative to ownship.
- `/cdrone/drone01/perception/world_tracks`
  Type: `drone_msgs/WorldTargetTrackArray`
  Meaning: map-frame target positions when ownship pose is available.
- `/cdrone/drone01/perception/status`
  Type: `drone_msgs/PerceptionStatus`
  Meaning: tracker rate, inference latency, detection count, active tracks.

## Build

From `cdrone_control/ros2`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source install/setup.bash
```

## Important Prerequisite

The new `tracking_only` path uses host RealSense access. It does not use the
container.

If host access is broken, the tracker launch will start but no camera frames
will arrive. Check it with:

```bash
/home/jetson/cdrone_control/scripts/check_realsense_host_access.sh
```

If `lsusb` sees the D455 but `rs-enumerate-devices` says `No device detected`,
install the vendored udev rules and replug the camera:

```bash
sudo /home/jetson/cdrone_control/scripts/install_realsense_host_access.sh
```

Bench note:

- If the D455 still refuses to enumerate on the host after the rule install,
  do a physical unplug/replug once before continuing.
- In my April 15, 2026 bench test, the host permission issue was fixed, but the
  camera eventually dropped off USB entirely after a failed software
  re-enumeration and required a manual replug to continue live testing.

The vendored rules file came from Intel RealSense upstream:

- `https://github.com/IntelRealSense/librealsense/blob/master/config/99-realsense-libusb.rules`

## Bench Validation Snapshot

Validated on April 15, 2026 with the D455 replugged and host udev rules
installed.

Observed with `tracking_only.launch.py` in the default direct mode:

- TensorRT engine loaded from
  `/home/jetson/cdrone_yolo/models/best_large_640_100e_77k_fp16.engine`
- `tracker_fps` stayed essentially at `10 Hz`
- `inference_latency_ms` was about `70 ms`
- body-frame tracks published continuously on
  `/cdrone/drone01/perception/tracks`
- GPU utilization rose during inference, confirming the detector is running on
  the GPU
- CPU load was materially lower than the earlier ROS-camera-plus-container path
  because the tracker now owns the camera directly

## Milestone 0: Bringup Sanity

Goal:
Confirm the ROS workspace builds and the new launch entry point resolves.

Command:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py
```

Pass criteria:

- `realsense_tracker_node` starts.
- The node logs `Started direct RealSense pipeline`.
- No package import or launch errors appear.

Notes:

- If you want to force the legacy ROS camera input path later for debugging, you
  can launch:

```bash
ros2 launch drone_bringup tracking_only.launch.py source_mode:=ros
```

## Milestone 1: Direct Camera Capture

Goal:
Confirm the tracker can open the D455 directly from the host and begin
processing live frames without the container.

Command:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py
```

In another terminal:

```bash
/home/jetson/cdrone_control/scripts/check_realsense_host_access.sh
```

Pass criteria:

- The launch logs `Started direct RealSense pipeline`.
- `check_realsense_host_access.sh` shows the D455 in `lsusb` and
  `rs-enumerate-devices`.
- The camera profile is stable enough that the Jetson is not swapping heavily.

Expected profile:

- Color: `848x480x15`
- Depth: `848x480x15`
- Aligned depth to color
- No IMU or stereo IR in `tracking_only`
- No `/camera/*` topics are expected in the default direct mode

## Milestone 2: Observe-Only Tracking

Goal:
Confirm the TensorRT detector and Norfair tracker run on live frames and publish
typed track messages.

Command:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 topic echo /cdrone/drone01/perception/status
ros2 topic echo /cdrone/drone01/perception/tracks
```

Pass criteria:

- `tracker_fps` is steady enough for bench testing.
- `inference_latency_ms` stays reasonable for the chosen engine.
- `TargetTrackArray` messages appear when another drone is visible.

Bench-validated sample:

- `tracker_fps`: about `10.0`
- `inference_latency_ms`: about `70`
- stable `track_id` observed across multiple consecutive messages

Design note:

- This node processes the latest frame at a fixed rate instead of trying to
  infer on every incoming frame.
- The default detector model auto-resolves from:
  - `/home/jetson/cdrone_yolo/models`
  - `cdrone_control/vendor/cdrone_yolo/models`
  - `cdrone_control/third_party/cdrone_yolo/models`
- You can override it explicitly with:

```bash
ros2 launch drone_vision_pkg tracking_only.launch.py model_path:=/abs/path/to/model.engine
```

## Milestone 3: Body-Frame Target Coordinates

Goal:
Verify that each published target has stable relative body-frame coordinates.

Topic:

- `/cdrone/drone01/perception/tracks`

Interpretation:

- `x_b_m`: meters in front of ownship
- `y_b_m`: meters left of ownship
- `z_b_m`: meters up relative to ownship

Command:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 topic echo /cdrone/drone01/perception/tracks
```

Pass criteria:

- A visible drone produces nonzero `x_b_m`, `y_b_m`, `z_b_m`.
- `distance_m` tracks motion sensibly.
- IDs remain reasonably stable while the target remains in view.

Bench-validated sample:

- `x_b_m`: about `1.20`
- `y_b_m`: about `0.07`
- `z_b_m`: about `0.14`
- `distance_m`: about `1.21`

If the coordinates are biased:

- Adjust `camera_offset_body_m`
- Adjust `camera_rpy_body_rad`

Those parameters live in:

- `ros2/src/drone_vision_pkg/config/tracking_only.yaml`

## Milestone 4: Map-Frame Target Coordinates

Goal:
Confirm the tracker can project targets into `map` once ownship pose is
available.

Default pose input for the tracker:

- `/cdrone/drone01/external_pose/input_pose`

That keeps the tracker decoupled from whether ownship pose currently comes from:

- OptiTrack through `external_pose_adapter_node`
- VSLAM later
- another future pose source

Quick check:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 topic echo /cdrone/drone01/perception/world_tracks
```

Pass criteria:

- `WorldTargetTrackArray` publishes when body-frame tracks exist and ownship pose
  is available.
- `frame_id` is `map` or the expected world frame.
- The world position moves consistently with the observed target.

If you need to point the tracker directly at another pose topic for bench work:

```bash
ros2 launch drone_vision_pkg tracking_only.launch.py \
  pose_topic:=/vrpn_mocap/RigidBody3/pose
```

## Milestone 5: Handoff to Control in Safe Steps

Goal:
Prepare the existing follow controller to consume body-frame tracks without
giving it authority too early.

Recommended order:

1. Run tracking only and record logs.
2. Run `target_follow.launch.py` with follow disabled on startup.
3. Confirm the controller sees `/cdrone/drone01/perception/tracks`.
4. Watch command outputs before enabling any live follow behavior.

Why this order:

- The controller already consumes `drone_msgs/TargetTrackArray`.
- The new world-track topic is for future planning and setpoint generation.
- We should validate perception stability before turning it into flight control.

## Milestone 6: Future Scaffolding

Available but not implemented yet:

```bash
ros2 launch drone_bringup vslam_only.launch.py
ros2 launch drone_bringup combined.launch.py
```

Current meaning:

- These are placeholders for the later milestones.
- `tracking_only` is the active implementation path right now.

## What To Use In Practice

Default end-to-end bringup:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_bringup tracking_only.launch.py
```

Advanced node-only bringup:

```bash
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
ros2 launch drone_vision_pkg tracking_only.launch.py
```

Use the node-only launch when you want to override parameters like `drone_id`,
`pose_topic`, or `model_path` without touching the default system wrapper.
