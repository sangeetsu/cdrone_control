# Level 2 Test 1 Implementation Summary

## Scope

This document summarizes the code changes implemented for the first Level 2 autonomy integration pass. The goal of this pass was to move the repo from:

- manual teleop plus bench-only offboard plumbing

to:

- a namespaced multi-drone-ready autonomy stack
- a follow-style autonomy state machine
- a VIO bridge contract for PX4 indoor aiding
- a telemetry aggregation layer
- a read-only remote terminal dashboard

This is an integration and scaffolding milestone. It does not yet include a real stereo VIO source inside this repo.

## High-Level Result

The repo now contains the following new system shape:

1. A per-drone topic layout based on `drone_id`.
2. A follow-oriented autonomy state machine instead of the previous illuminate/advance queue flow.
3. A `vio_bridge_node` that accepts an external pose stream and republishes it to MAVROS `vision_pose`.
4. A `telemetry_aggregator_node` that normalizes MAVROS, perception, and behavior state into a single `FlightStatus` stream plus alert messages.
5. A `dashboard_tui_node` that subscribes to monitor topics and displays a read-only terminal dashboard.
6. Extended launch/config plumbing so the autonomy stack can be started with `drone_id`, hostname, IP, and MAVROS namespace parameters.

## Files Added

### New messages

- `ros2/src/drone_msgs/msg/FlightStatus.msg`
- `ros2/src/drone_msgs/msg/SystemAlert.msg`
- `ros2/src/drone_msgs/msg/PerceptionStatus.msg`
- `ros2/src/drone_msgs/msg/PeerState.msg`

### New monitor package

- `ros2/src/drone_monitor_pkg/package.xml`
- `ros2/src/drone_monitor_pkg/setup.py`
- `ros2/src/drone_monitor_pkg/setup.cfg`
- `ros2/src/drone_monitor_pkg/resource/drone_monitor_pkg`
- `ros2/src/drone_monitor_pkg/drone_monitor_pkg/__init__.py`
- `ros2/src/drone_monitor_pkg/drone_monitor_pkg/topic_utils.py`
- `ros2/src/drone_monitor_pkg/drone_monitor_pkg/telemetry_aggregator_node.py`
- `ros2/src/drone_monitor_pkg/drone_monitor_pkg/dashboard_tui_node.py`

### New behavior/state-machine files

- `ros2/src/drone_behavior_pkg/drone_behavior_pkg/topic_utils.py`
- `ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/align.py`
- `ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/follow_standoff.py`
- `ros2/src/drone_behavior_pkg/drone_behavior_pkg/behaviors/lost_target_hold.py`
- `ros2/src/drone_behavior_pkg/config/scenarios/track_follow_v1.yaml`

### New control files

- `ros2/src/drone_control_pkg/drone_control_pkg/topic_utils.py`
- `ros2/src/drone_control_pkg/drone_control_pkg/vio_bridge_node.py`

### New launch file

- `ros2/src/drone_bringup/launch/dashboard.launch.py`

## Message and Interface Changes

### `drone_msgs`

The message package was extended so the stack can publish a stable dashboard/monitor contract instead of making the dashboard scrape many raw topics.

Added:

- `FlightStatus`
  - summarized pose, altitude, velocity, command output, mode, battery, autonomy state, target state, and freshness metrics
- `SystemAlert`
  - severity, code, message, latched flag
- `PerceptionStatus`
  - tracker FPS, inference latency, detection counts, active track count
- `PeerState`
  - per-drone identity and pose/velocity heartbeat for later swarm work

Changed:

- `EngagementState`
  - now includes:
    - `autonomy_enabled`
    - `autonomy_ready`
    - `blocked_reason`
    - `estop_latched`
    - `obstacle_blocked`
    - `min_distance_gate_active`
    - `active_bearing_rad`
    - `target_visible`

## Behavior Layer Changes

### State machine migration

The behavior layer was changed from the earlier target-illuminate flow:

- `SEARCH -> APPROACH -> ILLUMINATE -> ADVANCE_QUEUE`

to a pursuit/follow flow:

- `SEARCH -> ALIGN -> APPROACH -> FOLLOW_STANDOFF -> LOST_TARGET_HOLD -> FAILSAFE_HOLD`

### New behaviors

- `ALIGN`
  - rotates toward the target before closing
- `FOLLOW_STANDOFF`
  - maintains target at the configured standoff range
- `LOST_TARGET_HOLD`
  - publishes zero velocity during short reacquire window, then returns to `SEARCH`

### Updated behaviors

- `SEARCH`
  - now transitions to `ALIGN` instead of directly to `APPROACH`
- `APPROACH`
  - now drives toward `follow_distance_m`
  - transitions to `FOLLOW_STANDOFF` when target range is within tolerance
  - no longer performs dwell/illumination logic
- `FAILSAFE_HOLD`
  - remains the hard stop state

### `engagement_manager_node.py`

Major changes:

- added per-drone topic path parameters using `drone_id`
- added MAVROS namespace parameter support
- added `autonomy_enable` input topic
- added VIO freshness gating using `mavros/vision_pose/pose`
- added MAVROS state gating for `OFFBOARD` and armed state
- added simple obstacle stop logic using currently tracked objects inside a forward keepout volume
- added `min_distance_gate_active` tracking from the velocity controller math
- updated engagement-state publishing for dashboard consumption
- default scenario fallback now matches the follow-state machine

### `math_utils.py`

Added:

- `VelocityCommand` dataclass
- `target_bearing_rad()`
- `compute_velocity_command_details()`

The old `compute_velocity_command()` is still present as a compatibility wrapper, but the manager now uses the richer return type so it can expose min-distance gate state.

## Health and Safety Changes

### `health_monitor_node.py`

The health monitor now:

- uses per-drone namespaced topics
- listens for autonomy enable
- listens for VIO pose freshness
- only asserts estop automatically while autonomy is enabled
- includes VIO timeout in estop logic

This is meant to reduce false estops when the autonomy stack is not active while still giving a hard stop if the aiding or tracking pipeline goes stale during autonomy.

## Control Layer Changes

### `vio_bridge_node.py`

This node was added as the indoor aiding integration point.

Behavior:

- subscribes to `/cdrone/<drone_id>/vio/input_pose`
- republishes to `/<mavros_namespace>/vision_pose/pose`
- optionally publishes `CompanionProcessStatus`
- stops republishing if the input pose stream goes stale

This makes the VIO source pluggable. The repo still needs a real stereo VIO publisher to feed this topic.

### `mavros_velocity_node.py`

Added:

- `drone_id`
- `mavros_namespace`
- topic override parameters for command, estop, state, local pose, and output velocity topic

Changed:

- gate now accepts true `OFFBOARD` mode explicitly
- all MAVROS paths are parameterized instead of hard-coded

### `keyboard_teleop_node.py`

Added:

- `drone_id`
- `mavros_namespace`
- configurable control and estop topic paths

Effect:

- teleop remains usable, but now fits the per-drone namespaced architecture

### `bench_vision_pose_node.py`

Added:

- `mavros_namespace` parameter

Effect:

- bench-only vision pose publisher is still available, but no longer assumes root `/mavros`

### `light_controller_node.py`

Added:

- per-drone `light_cmd_topic`
- per-drone `estop_topic`

Effect:

- light control now follows the same namespaced autonomy layout

## Perception Layer Changes

### `stereo_tracker_node.py`

Added:

- `drone_id`
- namespaced `tracks_topic`
- namespaced `perception_status_topic`

New output:

- `PerceptionStatus`
  - publishes tracker FPS
  - publishes measured frame-loop latency
  - publishes left/right detection counts
  - publishes stereo pair count
  - publishes active track count

This is the main telemetry source for the new dashboard health pane.

## Monitoring and Dashboard Changes

### `telemetry_aggregator_node.py`

This is the main new monitoring node.

Inputs:

- MAVROS state
- MAVROS local pose
- MAVROS local body velocity
- MAVROS battery
- MAVROS vision pose freshness
- namespaced autonomy enable
- namespaced estop
- namespaced command velocity
- namespaced target tracks
- namespaced perception status
- namespaced engagement state

Outputs:

- `/cdrone/<drone_id>/monitor/flight_status`
- `/cdrone/<drone_id>/monitor/alerts`
- `/cdrone/<drone_id>/swarm/self_state`

Alerts currently generated:

- stale VIO
- stale tracks
- autonomy enabled but not in `OFFBOARD`
- autonomy blocked
- min distance gate active
- target lost
- low battery
- estop asserted

### `dashboard_tui_node.py`

This node provides a read-only terminal dashboard using `curses`.

Displayed panes:

- Flight
  - connected, armed, mode, altitude, pose, orientation, velocity, battery
- Autonomy
  - enabled, ready, blocked, reason, behavior state, target ID/range/bearing, command output, obstacle state, estop
- Health
  - VIO age, track age, MAVROS age, command age, watchdog state, min-distance gate, FPS, inference latency
- Alerts
  - current active latched alerts

It subscribes only. It does not publish command or control topics.

## Launch and Config Changes

### `autonomy_stack.launch.py`

This launch file now starts:

- MAVROS bringup
- stereo tracker
- VIO bridge
- engagement manager
- health monitor
- MAVROS velocity bridge
- light controller
- telemetry aggregator

New launch arguments:

- `drone_id`
- `hostname`
- `self_ip`
- `mavros_namespace`

Default scenario changed to:

- `track_follow_v1`

### `dashboard.launch.py`

New remote-UI launch for:

- `dashboard_tui_node`

This is intended to be run on the operator laptop after the ROS environment is sourced and the machine can see the target ROS graph.

### `drone.launch.py` and `drone_control_pkg/launch/mavros.launch.py`

Both now accept:

- `mavros_namespace`

This is required to avoid hard-coded `/mavros` assumptions.

### `autonomy_params.yaml`

Updated with:

- new per-drone defaults
- VIO bridge section
- follow-distance parameters
- obstacle stop parameters
- monitor parameters
- dashboard/host identity values

Also changed the default camera assumptions to match the current `cdrone_yolo` direction:

- left sensor `2`
- right sensor `3`
- local asset paths under `/home/sangeetsu/cdrone_yolo`

## Package Wiring Changes

### `drone_bringup/package.xml`

Added dependency:

- `drone_monitor_pkg`

### `drone_control_pkg`

Updated:

- `setup.py`
- `package.xml`

Added:

- `vio_bridge_node` entrypoint
- `sensor_msgs` dependency

### `drone_msgs/CMakeLists.txt`

Updated to generate the new monitor/swarm messages.

## Test Updates

### `test_behavior_transitions.py`

The behavior transition unit test was rewritten to match the new state machine:

- align to approach
- approach to follow
- follow persistence
- lost-target hold to search

## Verification Performed

### Passed

- Python syntax compilation succeeded for:
  - behavior package
  - control package
  - light package
  - monitor package
  - vision package
  - bringup launch files

### Not completed in this environment

- ROS interface generation build
- `colcon build`
- runtime node launch validation
- bench topic-level verification
- any props-off or props-on flight test

Reason:

- this environment does not have `/opt/ros/humble/setup.bash`, so a real ROS 2 build could not be executed here

## Current Limitations

The following are still not solved by this change set:

1. No real stereo VIO source exists in this repo.
2. The VIO bridge contract is implemented, but something external still has to publish `/cdrone/<drone_id>/vio/input_pose`.
3. The dashboard code exists, but it has not yet been run against live ROS topics.
4. The stack is namespaced and peer-state-ready, but full friendly/unknown filtering and decentralized swarm behavior are not yet implemented.
5. The obstacle model is only a simple stop/hold gate based on tracked objects in a forward corridor. There is no path planning or reroute behavior.
6. Battery telemetry depends on MAVROS publishing a valid battery topic in the deployed environment.

## Recommended Next Checkpoint

The next practical milestone is:

- get the repo to build in a real ROS 2 Humble environment
- feed a real pose stream into `vio_bridge_node`
- launch the autonomy stack and dashboard together
- verify topic names, freshness metrics, and blocked/autonomy-ready transitions on the bench before any flight attempt
