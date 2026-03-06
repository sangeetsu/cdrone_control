# cdrone_control

ROS2-based control and autonomy stack for a Jetson Orin Nano drone platform (ARK PAB carrier), with stereo tracking, multi-target engagement, MAVROS velocity control, and spotlight actuation.

## What This Repo Contains
This repository includes:

1. Core MAVROS bringup and control packages.
2. A modular autonomy stack for:
   - Stereo multi-object perception (`stereo_tracker_node`).
   - Target selection and engagement state machine (`engagement_manager_node`).
   - MAVROS-safe velocity forwarding (`mavros_velocity_node`).
   - Jetson GPIO/PWM spotlight control (`light_controller_node`).
   - Safety health monitoring (`health_monitor_node`).
3. Shared message definitions in `drone_msgs`.
4. Launch/config files for full autonomous execution.

## Runtime Assumptions

1. Runtime target: Jetson Orin Nano 8GB (Linux, ROS2 Humble), ARK custom kernel.
2. Flight stack bridge: MAVROS with ArduPilot GUIDED workflow.
3. Stereo cameras: dual IMX219, baseline ~60 mm.
4. Detection backend: TensorRT engine (primary), Norfair for track management.
5. Spotlight control: Jetson GPIO/PWM.
6. Optional camera backend: Intel RealSense (`source_mode:=realsense`).

## Quick Start (Jetson Runtime)

1. Install ROS2 + MAVROS dependencies.
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  python3-colcon-common-extensions \
  python3-pip \
  python3-opencv
```

2. Install GeographicLib datasets (required by MAVROS).
```bash
source /opt/ros/humble/setup.bash
ros2 run mavros install_geographiclib_datasets.sh
```

3. Install Python dependencies for Jetson runtime.
```bash
cd /path/to/cdrone_control
pip3 install -r requirements-jetson.txt
```

If using Intel RealSense, also install:
```bash
pip3 install pyrealsense2
```

4. Build the ROS2 workspace.
```bash
cd /path/to/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
```

5. Configure runtime paths in:
`ros2/src/drone_bringup/config/autonomy_params.yaml`
Set at least:
   - `model_path` (TensorRT engine path)
   - `calibration_path` (`stereo_calibration.npz`)
   - Camera source parameters (`source_mode`, sensor IDs or RTSP URLs)

6. Run full autonomy stack.
```bash
ros2 launch drone_bringup autonomy_stack.launch.py source_mode:=csi scenario:=intercept_illuminate_v1
```

## Common Launch Modes

1. Full stack (MAVROS + autonomy):
```bash
ros2 launch drone_bringup autonomy_stack.launch.py
```

2. MAVROS only:
```bash
ros2 launch drone_bringup drone.launch.py
```

3. RTSP stereo input mode:
```bash
ros2 launch drone_bringup autonomy_stack.launch.py source_mode:=rtsp
```

4. Explicit scenario file id:
```bash
ros2 launch drone_bringup autonomy_stack.launch.py scenario:=intercept_illuminate_v1
```

5. Intel RealSense mode:
```bash
ros2 launch drone_bringup autonomy_stack.launch.py source_mode:=realsense
```

## Safety Control Topics

1. E-stop assert:
```bash
ros2 topic pub /cdrone/safety/estop std_msgs/msg/Bool "{data: true}" --once
```

2. E-stop clear sequence:
```bash
ros2 topic pub /cdrone/safety/estop std_msgs/msg/Bool "{data: false}" --once
ros2 topic pub /cdrone/safety/estop_reset std_msgs/msg/Bool "{data: true}" --once
```

## File Hierarchy and Architecture

### Top-Level Layout
```text
cdrone_control/
|- docs/
|  |- ark_orin_multi_drone_plan.md
|- ros2/
|  |- src/
|     |- drone_bringup/
|     |- drone_msgs/
|     |- drone_vision_pkg/
|     |- drone_behavior_pkg/
|     |- drone_control_pkg/
|     |- drone_light_pkg/
|     |- ros2_poselib/
|- docker/
|- requirements-jetson.txt
|- requirements-dev.txt
```

### Package Roles

1. `drone_bringup`
   - Launch and global config.
   - Key files:
     - `launch/autonomy_stack.launch.py`
     - `launch/drone.launch.py`
     - `config/autonomy_params.yaml`
     - `config/apm_config.yaml`

2. `drone_msgs`
   - Message interfaces:
     - `TargetTrack.msg`
     - `TargetTrackArray.msg`
     - `EngagementState.msg`
     - `LightCommand.msg`

3. `drone_vision_pkg`
   - Perception and stereo tracking:
     - `drone_vision_pkg/stereo_tracker_node.py`
     - `drone_vision_pkg/stereo_utils.py`

4. `drone_behavior_pkg`
   - Engagement state machine and policy:
     - `drone_behavior_pkg/engagement_manager_node.py`
     - `drone_behavior_pkg/health_monitor_node.py`
     - `drone_behavior_pkg/behavior_registry.py`
     - `drone_behavior_pkg/behaviors/*`
     - `config/scenarios/intercept_illuminate_v1.yaml`

5. `drone_control_pkg`
   - MAVROS control adapters:
     - `drone_control_pkg/mavros_velocity_node.py`
     - Optional setup/control helpers (`drone_setup_node.py`, `drone_control_node.py`) remain available.

6. `drone_light_pkg`
   - Spotlight hardware output:
     - `drone_light_pkg/light_controller_node.py`
     - `drone_light_pkg/light_utils.py`

### Node Graph (Conceptual)

1. `stereo_tracker_node` publishes `/cdrone/perception/tracks`.
2. `engagement_manager_node` consumes tracks, runs scenario policy, publishes:
   - `/cdrone/control/cmd_vel_body`
   - `/cdrone/light/cmd`
   - `/cdrone/engagement/state`
3. `mavros_velocity_node` gates/sanitizes commands, forwards to:
   - `/mavros/setpoint_velocity/cmd_vel`
4. `light_controller_node` converts light commands to GPIO/PWM.
5. `health_monitor_node` supervises heartbeat freshness and asserts `/cdrone/safety/estop` on timeout.

## Important Config Files

1. `ros2/src/drone_bringup/config/autonomy_params.yaml`
   - Main autonomy tuning and hardware map.
2. `ros2/src/drone_behavior_pkg/config/scenarios/intercept_illuminate_v1.yaml`
   - Scenario state ordering and policy defaults.
3. `ros2/src/drone_bringup/config/apm_config.yaml`
   - MAVROS plugin config (`setpoint_velocity` frame set to `BODY_NED`).
4. `ros2/src/drone_control_pkg/config/apm_config.yaml`
   - Matching MAVROS settings for control package launch path.

## Development Notes (Windows + Jetson)

1. You can edit code on Windows.
2. Build and hardware-run on Jetson Linux.
3. Keep model engine and stereo calibration files on Jetson and point to them via `autonomy_params.yaml`.
4. Use `requirements-dev.txt` for local tooling and unit-test dependencies.

## Basic Verification

1. Syntax check:
```bash
python3 -m py_compile \
  ros2/src/drone_behavior_pkg/drone_behavior_pkg/*.py \
  ros2/src/drone_light_pkg/drone_light_pkg/*.py \
  ros2/src/drone_vision_pkg/drone_vision_pkg/*.py \
  ros2/src/drone_control_pkg/drone_control_pkg/*.py
```

2. Run lightweight math/unit tests (after installing `pytest`):
```bash
pip3 install -r requirements-dev.txt
python3 -m pytest -q
```
