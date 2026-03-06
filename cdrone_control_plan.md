# ARK Orin Multi-Drone Stereo Tracking and Sequential Illumination Plan

## Summary
Create a modular ROS2 autonomy stack in `cdrone_control` that runs on Jetson Orin Nano (ARK PAB custom kernel), uses dual IMX219 stereo + Norfair + TensorRT to track multiple drones, prioritizes targets with an inbound+nearest policy, flies to each target sequentially using MAVROS velocity control, and actuates a spotlight via Jetson GPIO/PWM.  
Place this plan in `docs/ark_orin_multi_drone_plan.md` as the editable implementation spec.

## Scope and Success Criteria
1. Detect and track multiple drones from stereo cameras using persistent Norfair IDs.
2. Estimate each target position in drone body frame (FRD) and update at >=10 Hz.
3. Select targets using inbound+nearest scoring and execute sequential engagement.
4. For each target, approach to configured range, keep light on for dwell time, mark complete, move to next.
5. Support modular scenario behavior via YAML scenario files + behavior registry.
6. Run on Jetson runtime (Linux/ARK kernel), with development on Windows.
7. Use TensorRT as primary inference runtime.
8. Use MAVROS velocity commands as primary control output.
9. Use Jetson GPIO/PWM for spotlight control.
10. Validation order is live-test-first (as requested), with hard pre-arm safety gates.

## Architecture (Decision-Complete)

### Nodes
| Node | Package | Responsibility |
|---|---|---|
| `stereo_tracker_node` | `drone_vision_pkg` | Dual camera ingest, rectification, detection, stereo matching, triangulation, Norfair tracking, body-frame conversion, publish tracks |
| `engagement_manager_node` | `drone_behavior_pkg` | Target scoring, queue management, scenario state machine, velocity/light command generation |
| `mavros_velocity_node` | `drone_control_pkg` | Mode/arm checks, watchdog, publish velocity commands to MAVROS, enforce control safety limits |
| `light_controller_node` | `drone_light_pkg` | Jetson GPIO/PWM spotlight actuation from ROS command topic |
| `health_monitor_node` | `drone_behavior_pkg` | Aggregate heartbeats and trigger hover/stop on node or topic timeout |

### Data Flow
1. `stereo_tracker_node` publishes target tracks in body frame.
2. `engagement_manager_node` ranks targets and runs scenario actions.
3. `engagement_manager_node` publishes `cmd_vel_body` and `light_cmd`.
4. `mavros_velocity_node` forwards `cmd_vel_body` to MAVROS velocity setpoint topic in body frame.
5. `light_controller_node` actuates GPIO/PWM.
6. `health_monitor_node` can override with zero-velocity and light-off safety command.

## Public APIs / Interfaces / Types

### New ROS2 Message Package
Create `ros2/src/drone_msgs` with these messages.

`TargetTrack.msg`
- `builtin_interfaces/Time stamp`
- `int32 track_id`
- `float32 x_b_m`
- `float32 y_b_m`
- `float32 z_b_m`
- `float32 vx_b_mps`
- `float32 vy_b_mps`
- `float32 vz_b_mps`
- `float32 distance_m`
- `float32 confidence`
- `float32 bbox_area_px`
- `bool inbound`

`TargetTrackArray.msg`
- `builtin_interfaces/Time stamp`
- `TargetTrack[] tracks`

`EngagementState.msg`
- `builtin_interfaces/Time stamp`
- `string scenario_id`
- `string state`
- `int32 active_track_id`
- `float32 active_distance_m`
- `float32 dwell_remaining_s`

`LightCommand.msg`
- `builtin_interfaces/Time stamp`
- `bool enabled`
- `float32 intensity_0_to_1`
- `float32 strobe_hz`

### ROS Topics
| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/cdrone/perception/tracks` | `drone_msgs/TargetTrackArray` | `stereo_tracker_node` | `engagement_manager_node`, `health_monitor_node` |
| `/cdrone/engagement/state` | `drone_msgs/EngagementState` | `engagement_manager_node` | Operator UI/loggers |
| `/cdrone/control/cmd_vel_body` | `geometry_msgs/TwistStamped` | `engagement_manager_node` | `mavros_velocity_node` |
| `/cdrone/light/cmd` | `drone_msgs/LightCommand` | `engagement_manager_node` | `light_controller_node` |
| `/cdrone/safety/estop` | `std_msgs/Bool` | Operator/health monitor | `engagement_manager_node`, `mavros_velocity_node`, `light_controller_node` |
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | `mavros_velocity_node` | MAVROS |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | `mavros_velocity_node` |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | MAVROS | `mavros_velocity_node`, `engagement_manager_node` |

### Config and Scenario Interfaces
1. `autonomy_params.yaml` contains system defaults and hardware mapping.
2. `scenarios/*.yaml` define behavior sequence and policy constants.
3. `behavior_registry.py` maps YAML action names to Python behavior classes.

## File-Level Implementation Plan

### Create
1. `ros2/src/drone_msgs/` package (`package.xml`, `CMakeLists.txt`, `msg/*.msg`).
2. `ros2/src/drone_behavior_pkg/` package with:
   - `drone_behavior_pkg/engagement_manager_node.py`
   - `drone_behavior_pkg/health_monitor_node.py`
   - `drone_behavior_pkg/behavior_registry.py`
   - `drone_behavior_pkg/behaviors/search.py`
   - `drone_behavior_pkg/behaviors/approach.py`
   - `drone_behavior_pkg/behaviors/illuminate.py`
   - `drone_behavior_pkg/behaviors/advance_queue.py`
   - `config/scenarios/intercept_illuminate_v1.yaml`
3. `ros2/src/drone_light_pkg/` package with `drone_light_pkg/light_controller_node.py`.
4. `ros2/src/drone_vision_pkg/drone_vision_pkg/stereo_tracker_node.py`.
5. `ros2/src/drone_control_pkg/drone_control_pkg/mavros_velocity_node.py`.
6. `ros2/src/drone_bringup/launch/autonomy_stack.launch.py`.
7. `ros2/src/drone_bringup/config/autonomy_params.yaml`.

### Modify
1. `ros2/src/drone_bringup/config/apm_config.yaml`:
   - Set `/mavros/**/setpoint_velocity/ros__parameters/mav_frame: BODY_NED`.
2. `ros2/src/drone_vision_pkg/setup.py` and `package.xml`:
   - Add `stereo_tracker_node` entrypoint and dependencies (`scipy`, `ultralytics`, `norfair`, `opencv-python`, `pycuda`, `tensorrt`).
3. `ros2/src/drone_control_pkg/setup.py` and `package.xml`:
   - Add `mavros_velocity_node` entrypoint and required deps.
4. `ros2/src/drone_bringup/launch/drone.launch.py`:
   - Keep MAVROS launch; do not include autonomy nodes there.
5. Add new launch option in `autonomy_stack.launch.py` for source mode (`csi` or `rtsp`).

## Core Algorithms and Behavior Definitions

### Stereo Tracker Pipeline
1. Input mode parameter `source_mode` supports `csi` and `rtsp`.
2. CSI defaults: `left_sensor_id=0`, `right_sensor_id=1`.
3. RTSP defaults: `left_url=rtsp://127.0.0.1:5600/camera1`, `right_url=rtsp://127.0.0.1:5600/camera2`.
4. Load calibration from `stereo_calibration.npz`.
5. Rectify both frames with `cv2.remap`.
6. Run TensorRT engine on each frame.
7. Match detections with epipolar gate `|yL-yR| <= 10 px` + Hungarian assignment on x-distance.
8. Triangulate with `cv2.triangulatePoints(P1, P2, ...)` to camera-frame 3D.
9. Convert camera frame to body frame with fixed extrinsic params.
10. Update Norfair tracker in 3D point space.
11. Publish `TargetTrackArray` with position, velocity estimate, confidence, inbound flag.

### Target Ranking (Inbound+Nearest)
Use:
- `radial_v = -dot(p, v) / (||p|| + 1e-3)` in body frame.
- `inbound_component = max(0, radial_v)`.
- `distance_component = 1 / (distance_m + 0.1)`.
- `score = 0.7 * inbound_component + 0.3 * distance_component`.
Select highest score not in cooldown set.

### Scenario Engine (YAML + Registry)
Default scenario `intercept_illuminate_v1` states:
1. `SEARCH` picks active target from ranked queue.
2. `APPROACH` commands velocity until `distance_m <= engage_distance_m`.
3. `ILLUMINATE` sends light on and holds for `dwell_time_s` while maintaining range.
4. `ADVANCE_QUEUE` marks target complete, starts cooldown, selects next.
5. `FAILSAFE_HOLD` zero velocity and light off on any safety trigger.

### Engagement Completion Rule
A target is complete when all are true:
1. `distance_m <= engage_distance_m`.
2. Track is continuously visible for `stability_time_s`.
3. Light has remained enabled for `dwell_time_s`.

## Safety and Control Rules
1. Hard minimum distance: if `distance_m < min_safe_distance_m`, command zero forward velocity.
2. Command watchdog: if no fresh velocity command for `>0.5 s`, send zero velocity.
3. Lost target timeout: if active target lost for `>0.75 s`, enter `FAILSAFE_HOLD`.
4. Reacquire timeout: if not reacquired within `2.0 s`, drop target and return `SEARCH`.
5. E-stop topic immediately sets zero velocity, disables light, and latches until manual reset.
6. Mode gate: commands publish only when MAVROS mode is `GUIDED` and armed.
7. Clamp limits:
   - `max_vel_xy_mps = 1.5`
   - `max_vel_z_mps = 0.8`
   - `max_yaw_rate_rps = 0.6`

## Default Parameters
| Parameter | Default |
|---|---|
| `follow_distance_m` | `3.0` |
| `engage_distance_m` | `1.2` |
| `dwell_time_s` | `2.0` |
| `stability_time_s` | `0.5` |
| `target_cooldown_s` | `15.0` |
| `min_safe_distance_m` | `0.8` |
| `detection_conf_threshold` | `0.35` |
| `tracker_rate_hz` | `20` |
| `control_rate_hz` | `20` |
| `light_gpio_pin` | `33` |
| `light_pwm_hz` | `200` |
| `light_default_intensity` | `0.9` |
| `baseline_mm` | `60` |

## Launch and Deployment
1. Build on Jetson with ROS2 Humble workspace under Linux.
2. Launch command:
   - `ros2 launch drone_bringup autonomy_stack.launch.py source_mode:=csi scenario:=intercept_illuminate_v1`
3. Keep MAVROS in existing bringup path; autonomy stack launch composes MAVROS + new nodes.
4. Development on Windows remains code-only; hardware test loop runs on Jetson.
5. Include `requirements-jetson.txt` and `requirements-dev.txt` for reproducible installs.

## Test Cases and Scenarios

### Unit Tests
1. Stereo matcher returns deterministic pairings for synthetic bounding boxes.
2. Triangulation returns correct depth within tolerance for known disparity cases.
3. Target scoring selects inbound closer target over outbound near target.
4. Scenario transitions follow YAML sequence and guard conditions.
5. GPIO adapter maps `LightCommand` intensity to PWM duty cycle correctly.

### Integration Tests
1. Perception publishes >=10 Hz with two cameras and non-empty track IDs when target is visible.
2. Track ID continuity survives 0.5 s occlusion.
3. Engagement manager advances through at least two targets sequentially without manual reset.
4. MAVROS adapter suppresses commands when not in `GUIDED`/armed.
5. E-stop forces zero velocity and light off within one control cycle.

### Live-Test-First Sequence (Requested)
1. Start with constrained live hover at low altitude with one target and one observer.
2. Validate one full approach + illuminate cycle.
3. Introduce second target and confirm queue advancement.
4. Validate inbound+nearest priority with moving inbound/outbound targets.
5. Run 10-cycle endurance test with no watchdog violations.

## Acceptance Criteria
1. System tracks at least 3 targets simultaneously in lab airspace.
2. At least 90% of engagements complete with correct range+dwell behavior over 20 attempts.
3. No unsafe forward command when distance violates minimum safe distance.
4. Full autonomy stack recovers from single target loss and continues queue processing.
5. Scenario can be swapped by changing YAML file only, with no code edits.

## Assumptions and Defaults Chosen
1. Runtime ROS distro is Humble.
2. Dual IMX219 baseline is fixed at 60 mm and calibration file is already valid.
3. TensorRT engines are prebuilt and available on Jetson filesystem.
4. Camera source defaults to CSI pair; RTSP remains selectable for compatibility.
5. Spotlight hardware is directly driven by Jetson GPIO/PWM, not FCU relay.
6. MAVROS remains the flight-control bridge and BODY_NED velocity commands are accepted.
7. Legacy nodes in `drone_controller_cpp` and old Python controller remain untouched during MVP; new autonomy launch is the default runtime path.
