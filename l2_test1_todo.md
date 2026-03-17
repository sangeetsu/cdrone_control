# Level 2 Test 1 TODO

## Immediate Build/Environment Tasks

- Install or source a real ROS 2 Humble environment on the target machine.
- Run a full `colcon build` for:
  - `drone_msgs`
  - `drone_behavior_pkg`
  - `drone_control_pkg`
  - `drone_light_pkg`
  - `drone_monitor_pkg`
  - `drone_vision_pkg`
  - `drone_bringup`
- Fix any ROS message-generation or package dependency issues that appear during the first real build.
- Confirm `mavros_msgs`, `sensor_msgs`, `geometry_msgs`, and `BatteryState` interfaces are available in the deployed workspace.

## VIO Integration Tasks

- Choose the actual stereo VIO source that will feed `/cdrone/<drone_id>/vio/input_pose`.
- Implement or integrate the external VIO publisher.
- Confirm that the VIO pose frame matches PX4/MAVROS expectations.
- Validate pose timestamps and update rate.
- Verify `vio_bridge_node` republishes cleanly to `/<mavros_namespace>/vision_pose/pose`.
- Confirm PX4 accepts the external vision stream and remains healthy in `OFFBOARD`.

## Topic and Namespacing Validation

- Launch the autonomy stack with a real `drone_id` and verify all namespaced topics resolve correctly.
- Confirm MAVROS namespace behavior for:
  - state
  - local pose
  - local body velocity
  - vision pose
  - battery
  - setpoint velocity
- Confirm teleop still works after namespacing changes.
- Verify the dashboard can subscribe from the remote operator laptop.

## Perception Validation

- Confirm `stereo_tracker_node` runs with the current local asset paths:
  - `/home/sangeetsu/cdrone_yolo/non_xai_best.engine`
  - `/home/sangeetsu/cdrone_yolo/stereo_calibration.npz`
- Verify the current CSI sensor IDs are correct on the target hardware.
- Validate `PerceptionStatus` values:
  - FPS
  - inference latency
  - left/right detections
  - stereo pair count
  - active track count
- Check whether the camera-to-body extrinsics are still identity in config and replace them with measured values if not.

## Autonomy Logic Validation

- Bench-test the new state machine transitions:
  - `SEARCH -> ALIGN`
  - `ALIGN -> APPROACH`
  - `APPROACH -> FOLLOW_STANDOFF`
  - `FOLLOW_STANDOFF -> LOST_TARGET_HOLD`
  - `LOST_TARGET_HOLD -> SEARCH`
  - `FAILSAFE_HOLD`
- Verify autonomy stays blocked when:
  - autonomy enable is false
  - VIO is stale
  - MAVROS is disconnected
  - drone is not armed
  - mode is not `OFFBOARD`
- Verify the manager publishes zero velocity while blocked.
- Verify min-distance gate is exposed correctly in `EngagementState` and `FlightStatus`.

## Health and Alert Validation

- Confirm `health_monitor_node` only asserts estop while autonomy is enabled.
- Force stale tracks and verify estop assertion.
- Force stale VIO and verify estop assertion.
- Force recovery and verify estop clear behavior.
- Verify alert generation in `telemetry_aggregator_node` for:
  - stale VIO
  - stale tracks
  - offboard rejected
  - autonomy blocked
  - min distance gate
  - target lost
  - battery low
  - estop

## Dashboard Validation

- Run `dashboard_tui_node` on the remote laptop.
- Confirm the dashboard renders correctly over SSH/local terminal.
- Validate each pane against live data:
  - Flight
  - Autonomy
  - Health
  - Alerts
- Confirm the dashboard remains stable if:
  - telemetry topics stop temporarily
  - autonomy stack nodes restart
  - alert states latch and clear

## Bench Test Sequence

- Bring up MAVROS only and confirm connection.
- Bring up bench VIO or external VIO source and confirm `vision_pose` freshness.
- Bring up autonomy stack and watch `FlightStatus`.
- Confirm:
  - autonomy blocked reason changes as expected
  - autonomy ready becomes true only when all preconditions are satisfied
  - command watchdog age behaves correctly
  - tracks age behaves correctly
- Use bench target motion or bag replay to exercise follow behavior without flight.

## Flight Test Prerequisites

- Confirm arming issue root cause is resolved on actual hardware.
- Confirm PX4 holds `OFFBOARD` with real external vision for at least 2 minutes.
- Validate manual takeover path back to `ALTCTL`.
- Confirm estop topic can stop command output immediately.
- Confirm obstacle stop logic does not false-trigger under normal tracking conditions.
- Confirm there is a spotter and clear test area before any props-on attempt.

## Known Code Cleanup Still Needed

- Update stale log strings that still reference old hard-coded topic names in a few places.
- Decide whether old illuminate/advance queue behavior files should remain in the repo or be removed.
- Decide whether `intercept_illuminate_v1.yaml` should remain as a compatibility scenario or be renamed/deprecated.
- Update `README.md` and operator docs to reflect:
  - new `track_follow_v1` default
  - new monitor package
  - new dashboard launch
  - VIO bridge contract
  - namespaced topic model
- Add unit tests for:
  - telemetry aggregation
  - alert latching/clearing
  - autonomy block reasons
  - VIO timeout logic

## Swarm/Level 4 Work Not Yet Implemented

- peer-to-peer state subscription and fusion
- friendly vs unknown target matching
- nearest unknown target selection
- dynamic keepout around known friendlies
- multi-drone dashboard summary rows
- decentralized assignment logic

## Suggested First Commands On A Real Target Machine

```bash
# Build
cd /home/sangeetsu/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Bring up stack
ros2 launch drone_bringup autonomy_stack.launch.py drone_id:=drone01 mavros_namespace:=mavros

# Bring up remote dashboard
ros2 launch drone_bringup dashboard.launch.py drone_id:=drone01

# Inspect monitor output
ros2 topic echo /cdrone/drone01/monitor/flight_status
ros2 topic echo /cdrone/drone01/monitor/alerts
```
