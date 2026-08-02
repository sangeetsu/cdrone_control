# Minimum-Jerk Mocap vs Optical-Flow/IMU-Fusion Runbook

This runbook launches an additive test path:

- waypoint flight uses mocap/PX4 external-vision position for control
- host-side PX4Flow dead-reckon is observational
- host-side IMU-only propagation is observational
- host-side IMU+PX4Flow fusion is observational
- mocap is used to initialize estimator origins when the mission becomes
  active; after that, mocap is only ground truth for comparison
- mocap, optical-flow, IMU-only, and fused X, Y, and Z estimates are compared
  over the same timestamps in the same `map` frame
- output CSV and stats are written under `flight_logs/of_compare/<run_id>/`

The fused estimator is dynamic: a nine-state position/velocity/accelerometer-
bias EKF changes its gain from covariance, sensor quality, range, tilt, timing
coverage, gyro source, and measurement innovation. It rejects stale,
out-of-order, excessive-tilt, and innovation-gated measurements instead of
applying a fixed blend on every callback. The legacy fixed-weight launch
arguments remain accepted for compatibility but are deprecated.

## Why `minjerk_of_004` Was Poor

The recording shows an input-observability problem in addition to estimator
tuning: optical flow arrived at about 9.9 Hz while each sample covered only
15.872 ms, leaving most elapsed time unobserved. Flow and mocap velocity had
almost no directional correlation, flow gyro integrals were unavailable, and
IMU acceleration was vibration dominated. On identical valid rows, the old
fixed-weight fusion only changed XY RMSE from about 10.05 m to 9.94 m and made Z
worse. IMU-only double integration drifts much farther and is included as an
honest diagnostic baseline, not as a navigation solution.

Do not compensate this delivery gap by multiplying flow by `6.3` (or any other
fixed factor). Request the full/native sensor stream, retain source timestamps
and integration windows, then fix frame/extrinsic signs and signal correlation
before tuning covariance.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Waypoints

The default mission uses a start-aware full-studio sweep:

```bash
/home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints.yaml
```

The file uses absolute `map` coordinates and opts into dynamic route entry:

```yaml
route:
  start_policy: nearest_reachable
  loop: true
  return_to_start: true

waypoints:
  - name: south_west_sweep
    x_m: -12.20
    y_m: -2.35
    z_m: 1.80
    yaw_deg: 0.0
```

With `start_policy: nearest_reachable`, the waypoint mission chooses the
nearest waypoint that has a perimeter-safe straight-line entry from the drone's
actual start pose, rotates the cyclic route to begin there, completes the full
sweep, then returns to the actual start XY before landing. This makes the route
tolerant of any takeoff location that is already inside the guarded studio safe
area and outside the pillar keep-out.

Keep `frame_id: map` unless the mocap system is intentionally publishing a
different frame. The mission node refuses to start if waypoint, mocap, local
pose, or perimeter frames disagree.

The perimeter guard remains enabled for this runbook. Starts outside the green
studio safety area, starts inside the keep-out, unsafe waypoint legs, or starts
with no reachable entry waypoint are rejected before arming.

For a longer drift check, use the optional repeated-box mission:

```bash
/home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints_drift.yaml
```

It is centered near the `cdrone3` studio start area observed in
`minjerk_of_001`. Confirm the current mocap pose is still in that open area
before using it.

## Props-Off Frame Sanity Check

Like Milestone 3, start the launch stack first and keep it running. This is the
command that brings up MAVROS, VRPN/mocap, the external-pose bridge, the
minimum-jerk mission node, and the mocap-vs-OF logger:

If MAVLink routing is not already running, start it in its own terminal and
leave it running:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

Then start the minjerk comparison stack in another terminal:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
scripts/run_minjerk_of_compare.sh --run-id props_off_frame_check
```

In a second terminal, confirm the launch is visible before any service call:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
ros2 topic list
```

At minimum, expect topics under `/cdrone/cdrone3/mavros`, the mocap pose topic,
`/cdrone/cdrone3/demo/minjerk_waypoints_state`, and
`/cdrone/cdrone3/of_compare/pose`. Also expect
`/cdrone/cdrone3/of_compare/imu_only_pose` and
`/cdrone/cdrone3/of_compare/fused_pose`. If only `/parameter_events` and
`/rosout` appear, the launch stack is not visible in that terminal.

Then verify the PX4Flow topics:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
scripts/check_px4flow_topics.sh --request-stream-rate 70
```

The checker requests the 70 Hz native-rate target by default and reports the
observed delivery rate, delivery period, sensor integration window, and their
coverage ratio. Use `--no-request-stream` only when the stream rate must remain
untouched. Treat coverage below 80% as a data-gap failure to correct before
flight; the actual attainable rate remains sensor/autopilot dependent.

Confirm the MAVROS IMU topic is present. The default configured topic is:

```bash
ros2 topic echo /cdrone/cdrone3/mavros/imu/data --once
```

With the launch still running, watch mocap, OF, IMU-only, and fused pose:

```bash
ros2 topic echo /cdrone/cdrone3/vrpn_mocap/RigidBody3/pose --once
ros2 topic echo /cdrone/cdrone3/of_compare/pose --once
ros2 topic echo /cdrone/cdrone3/of_compare/imu_only_pose --once
ros2 topic echo /cdrone/cdrone3/of_compare/fused_pose --once
```

Move the drone by hand in mocap +X, then +Y. The OF and fused poses should
change with the same signs after the logger receives PX4Flow samples. The
IMU-only pose will drift and is not used for this sign check. If signs are
wrong, rerun with launch overrides until signs match:

```bash
scripts/run_minjerk_of_compare.sh --run-id flow_sign_test -- \
  flow_scale_x:=-1.0 \
  flow_scale_y:=1.0 \
  fusion_flow_sensor_yaw_rad:=0.0
```

## Launch

This is the mission bringup command. Do this before the start service call and
leave it running:

If MAVLink routing is not already running, start it in its own terminal first:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
scripts/run_minjerk_of_compare.sh --run-id minjerk_of_001
```

Useful overrides:

```bash
scripts/run_minjerk_of_compare.sh \
  --waypoints-config /home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints.yaml \
  --output-dir /home/jetson/cdrone_control/flight_logs/of_compare \
  --run-id minjerk_of_001 \
  -- \
  quality_min:=20 \
  range_min_m:=0.3 \
  range_max_m:=4.5 \
  imu_topic:=/cdrone/cdrone3/mavros/imu/data \
  fusion_enabled:=true
```

EKF tuning overrides can be passed the same way:

```bash
scripts/run_minjerk_of_compare.sh --run-id minjerk_fusion_tuned_001 -- \
  fusion_accel_noise_mps2:=0.80 \
  fusion_accel_bias_rw_mps3:=0.05 \
  fusion_flow_velocity_noise_mps:=0.25 \
  fusion_range_noise_m:=0.12 \
  fusion_innovation_gate_nis:=9.21 \
  fusion_max_sensor_age_s:=0.15
```

Timing, health, and extrinsic controls include
`fusion_reorder_tolerance_s`, `fusion_max_tilt_rad`,
`fusion_max_flow_gap_s`, `fusion_flow_gap_noise_scale`,
`fusion_flow_quality_noise_scale`, `fusion_flow_range_noise_scale`,
`fusion_gyro_fallback_noise_scale`, `fusion_gyro_coverage_tolerance_s`,
`fusion_gyro_buffer_duration_s`, and `fusion_flow_sensor_yaw_rad`. Calibrate
`flow_scale_x`, `flow_scale_y`, and the sensor yaw on a props-off translated and
rotated dataset before covariance tuning.

The old `fusion_flow_position_weight`, `fusion_flow_velocity_weight`,
and `fusion_range_z_weight` arguments remain launch-compatible as deprecated
variance multipliers, not direct complementary blends. New tuning should use
noise, initial uncertainty, timing, and innovation-gate parameters.

Drift-test launch using the longer optional waypoint set:

```bash
scripts/run_minjerk_of_compare.sh \
  --waypoints-config /home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints_drift.yaml \
  --run-id minjerk_of_drift_001
```

## RigidBody4 Optical-Flow Waypoint Flight

Use this section for the physical `cdrone4` vehicle with OptiTrack
`RigidBody4`. Waypoint control uses PX4 external vision from mocap only;
PX4Flow, VIO, IMU-only, IMU+PX4Flow fusion, and ZED images are observational
logs and must not publish to PX4's `/mavros/vision_pose/pose` input.

The launch requests native practical data rates automatically:
`HIGHRES_IMU` and `ATTITUDE_QUATERNION` at 250 Hz, and
`OPTICAL_FLOW_RAD`/`OPTICAL_FLOW` at 70 Hz. Keep the PX4Flow sensor at least
`0.3 m` above the surface before trusting `range_min_m:=0.3` flow/fusion
validity.

The ZED 2i records the left color image at 2 Hz as JPEG files under:

```text
flight_logs/of_compare/${RUN_ID}/images_dataset/
flight_logs/of_compare/${RUN_ID}/images_dataset/images.csv
```

The comparison CSV is timer-sampled at `logger_publish_rate_hz`, but raw IMU
and optical-flow messages are also written directly from their ROS callbacks.
Those raw logs preserve the variable source rate MAVROS is delivering at that
instant:

```text
flight_logs/of_compare/${RUN_ID}/imu_raw.csv
flight_logs/of_compare/${RUN_ID}/optical_flow_raw.csv
flight_logs/of_compare/${RUN_ID}/state_raw.csv
flight_logs/of_compare/${RUN_ID}/mocap_raw.csv
flight_logs/of_compare/${RUN_ID}/px4_vision_pose_raw.csv
flight_logs/of_compare/${RUN_ID}/px4_local_pose_raw.csv
```

RigidBody4 mocap is rotated by the external-pose bridge with a yaw of `pi`.
Use the transformed waypoint file with the normal studio perimeter:

```text
ros2/src/drone_bringup/config/minjerk_waypoints_rigidbody4_external.yaml
ros2/src/drone_bringup/config/drone_studio_perimeter.yaml
```

The raw mirrored RigidBody4 files are only for a no-transform setup:
`minjerk_waypoints_rigidbody4.yaml` and
`drone_studio_perimeter_rigidbody4.yaml`.

For the first cdrone4 drift-stabilization run, use the indoor profile:

```text
ros2/src/drone_bringup/config/indoor_speed_profile.yaml
```

This profile limits horizontal cruise, max velocity, acceleration, and jerk. It
does not change low-level PX4 position-loop gains, and the mission node should
restore the FCU's original values on exit.

### Copy/Paste cdrone4 Run Commands

Terminal 1, MAVLink router:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

Terminal 2, build and launch:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_bringup

cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

RUN_ID="minjerk_of_cdrone4_rb4_of_$(date -u +%Y%m%dT%H%M%SZ)"

scripts/run_minjerk_of_compare.sh \
  --waypoints-config /home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints_rigidbody4_external.yaml \
  --output-dir /home/jetson/cdrone_control/flight_logs/of_compare \
  --run-id "${RUN_ID}" \
  --request-sensor-stream-rates \
  --imu-stream-rate-hz 250 \
  --flow-stream-rate-hz 70 \
  --capture-images \
  --image-rate-hz 2 \
  --image-format jpg \
  -- \
  drone_id:=cdrone4 \
  drone_namespace:=/cdrone/cdrone4 \
  mavros_namespace:=/cdrone/cdrone4/mavros \
  mocap_namespace:=/cdrone/cdrone4/vrpn_mocap \
  rigid_body_name:=RigidBody4 \
  source_pose_topic:=/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose \
  mocap_pose_topic:=/cdrone/cdrone4/external_pose/input_pose \
  frame_rpy_rad:="[0.0, 0.0, 3.141592653589793]" \
  perimeter_config:=/home/jetson/cdrone_control/ros2/src/drone_bringup/config/drone_studio_perimeter.yaml \
  setpoint_topic:=/cdrone/cdrone4/mavros/setpoint_position/local \
  mission_state_topic:=/cdrone/cdrone4/demo/minjerk_waypoints_state \
  mavros_state_topic:=/cdrone/cdrone4/mavros/state \
  px4_local_pose_topic:=/cdrone/cdrone4/mavros/local_position/pose \
  px4_vision_pose_topic:=/cdrone/cdrone4/mavros/vision_pose/pose \
  start_service:=/cdrone/cdrone4/demo/minjerk_waypoints_start \
  abort_service:=/cdrone/cdrone4/demo/minjerk_waypoints_abort \
  flow_rad_topic:=/cdrone/cdrone4/mavros/px4flow/raw/optical_flow_rad \
  flow_range_topic:=/cdrone/cdrone4/mavros/px4flow/ground_distance \
  imu_topic:=/cdrone/cdrone4/mavros/imu/data \
  fused_pose_topic:=/cdrone/cdrone4/of_compare/fused_pose \
  imu_only_pose_topic:=/cdrone/cdrone4/of_compare/imu_only_pose \
  use_speed_profile:=true \
  speed_profile_config:=/home/jetson/cdrone_control/ros2/src/drone_bringup/config/indoor_speed_profile.yaml \
  restore_speed_profile_on_exit:=true \
  quality_min:=20 \
  range_min_m:=0.3 \
  range_max_m:=4.5 \
  fusion_enabled:=true \
  imu_only_enabled:=true \
  fusion_max_sensor_age_s:=0.15 \
  fusion_max_flow_gap_s:=0.25
```

Terminal 3, pre-start checks:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash

ros2 topic list | rg '^/cdrone/cdrone4/'

lsusb | rg -i 'stereolabs|zed'
lsusb -t | rg '5000M|STEREOLABS|ZED|uvcvideo'
python3 -c "import pyzed.sl as sl; print(sl.Camera.get_sdk_version())"

ros2 param get /mavlink_stream_rate_node enabled
ros2 param get /mavlink_stream_rate_node imu_rate_hz
ros2 param get /mavlink_stream_rate_node flow_rate_hz
ros2 param get /zed_image_capture_node enabled
ros2 param get /zed_image_capture_node rate_hz
ros2 param get /zed_image_capture_node image_format

ros2 topic echo --once /cdrone/cdrone4/demo/minjerk_waypoints_state
ros2 topic echo --once /cdrone/cdrone4/mavros/state
ros2 topic echo --once /cdrone/cdrone4/mavros/local_position/pose
ros2 topic echo --once /cdrone/cdrone4/mavros/vision_pose/pose
timeout 10 ros2 topic hz /cdrone/cdrone4/mavros/imu/data
scripts/check_px4flow_topics.sh --mavros-namespace /cdrone/cdrone4/mavros --no-request-stream --duration 10
ros2 topic echo --once /cdrone/cdrone4/mavros/px4flow/raw/optical_flow_rad
ros2 topic echo --once /cdrone/cdrone4/mavros/px4flow/ground_distance
ros2 topic echo --once /cdrone/cdrone4/external_pose/debug/summary
ros2 topic echo --once /cdrone/cdrone4/external_pose/input_pose
ros2 node info /cdrone/cdrone4/external_pose_bridge_node | sed -n '/Subscribers:/,/Publishers:/p'
ros2 param get /cdrone/cdrone4/mavros/param EKF2_EV_CTRL
ros2 param get /cdrone/cdrone4/mavros/param EKF2_OF_CTRL
ros2 topic echo --once /cdrone/cdrone4/of_compare/fused_pose
ros2 topic echo --once /cdrone/cdrone4/of_compare/imu_only_pose
ros2 param get /minjerk_waypoint_mission_node waypoints_config
ros2 param get /minjerk_waypoint_mission_node perimeter_config
ros2 param get /minjerk_waypoint_mission_node speed_profile_config
ros2 param get /mocap_of_compare_logger_node fusion_enabled
ros2 param get /mocap_of_compare_logger_node imu_only_enabled
ros2 param get /mocap_of_compare_logger_node mavros_state_topic
ros2 param get /mocap_of_compare_logger_node px4_local_pose_topic
ros2 param get /mocap_of_compare_logger_node px4_vision_pose_topic

ls -lh "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/images_dataset" | tail
sed -n '1,5p' "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/images_dataset/images.csv"
```

Expected before start:

- mission state is `IDLE`
- MAVROS is `connected: true`, `armed: false`, and normally `AUTO.LOITER`
- RigidBody4 external pose is fresh, transformed to frame `map`, and has no
  source timeout or source-gap warnings
- `/cdrone/cdrone4/external_pose_bridge_node` subscribes to
  `/cdrone/cdrone4/external_pose/input_pose` only for PX4 vision input; it
  must not subscribe to `/cdrone/cdrone4/vio/input_pose`
- PX4 estimator aiding is mocap/external-vision only for position control:
  `EKF2_EV_CTRL` is enabled for external vision and `EKF2_OF_CTRL` is `0`.
  PX4's onboard IMU remains part of its internal attitude estimator; the ROS
  `/mavros/imu/data` topic is only logged by this run.
- the ZED 2i appears on USB3 at `5000M`, the SDK import prints a version, and
  `/zed_image_capture_node` has `enabled=True`, `rate_hz=2.0`, and
  `image_format=jpg`
- PX4Flow raw and range topics pass the checker, preferably with at least 80%
  integration coverage after the launch-time 70 Hz stream request
- `/cdrone/cdrone4/mavros/imu/data` is materially above the old 20 Hz logger
  rate after the launch-time 250 Hz IMU stream request
- PX4Flow quality is above `quality_min` and range is finite; range may be
  rejected while the vehicle is sitting below `0.3 m`, but should become valid
  after takeoff
- image files and `images.csv` are appearing under
  `flight_logs/of_compare/${RUN_ID}/images_dataset/`
- mission waypoints are `minjerk_waypoints_rigidbody4_external.yaml`
- perimeter is `drone_studio_perimeter.yaml`
- speed profile is `indoor_speed_profile.yaml`
- logger has `fusion_enabled=True` and `imu_only_enabled=True`

### Manual STABILIZED Hover Diagnostics

The previous hover attempt did not fly in `STABILIZED`; it armed in
`AUTO.TAKEOFF`, aborted during takeoff, briefly requested `AUTO.LAND`, was
force-disarmed, and later returned to `POSCTL`. Do not run the autonomous
waypoint follower in `STABILIZED`: that mode does not follow ROS position
setpoints or hold altitude/position autonomously.

Use this diagnostic to log the same sensors and PX4 pose paths while the
operator manually flies a short STABILIZED hover. Launch the stack, keep the
mission state at `IDLE`, and do not call the start service.

```bash
RUN_ID="stabilized_hover_cdrone4_rb4_diag_$(date -u +%Y%m%dT%H%M%SZ)"

scripts/run_minjerk_of_compare.sh \
  --waypoints-config /home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints_rigidbody4_external.yaml \
  --output-dir /home/jetson/cdrone_control/flight_logs/of_compare \
  --run-id "${RUN_ID}" \
  --request-sensor-stream-rates \
  --imu-stream-rate-hz 250 \
  --flow-stream-rate-hz 70 \
  --capture-images \
  --image-rate-hz 2 \
  --image-format jpg \
  -- \
  drone_id:=cdrone4 \
  drone_namespace:=/cdrone/cdrone4 \
  mavros_namespace:=/cdrone/cdrone4/mavros \
  mocap_namespace:=/cdrone/cdrone4/vrpn_mocap \
  rigid_body_name:=RigidBody4 \
  source_pose_topic:=/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose \
  mocap_pose_topic:=/cdrone/cdrone4/external_pose/input_pose \
  frame_rpy_rad:="[0.0, 0.0, 3.141592653589793]" \
  perimeter_config:=/home/jetson/cdrone_control/ros2/src/drone_bringup/config/drone_studio_perimeter.yaml \
  setpoint_topic:=/cdrone/cdrone4/mavros/setpoint_position/local \
  mission_state_topic:=/cdrone/cdrone4/demo/minjerk_waypoints_state \
  mavros_state_topic:=/cdrone/cdrone4/mavros/state \
  px4_local_pose_topic:=/cdrone/cdrone4/mavros/local_position/pose \
  px4_vision_pose_topic:=/cdrone/cdrone4/mavros/vision_pose/pose \
  start_service:=/cdrone/cdrone4/demo/minjerk_waypoints_start \
  abort_service:=/cdrone/cdrone4/demo/minjerk_waypoints_abort \
  flow_rad_topic:=/cdrone/cdrone4/mavros/px4flow/raw/optical_flow_rad \
  flow_range_topic:=/cdrone/cdrone4/mavros/px4flow/ground_distance \
  imu_topic:=/cdrone/cdrone4/mavros/imu/data \
  fused_pose_topic:=/cdrone/cdrone4/of_compare/fused_pose \
  imu_only_pose_topic:=/cdrone/cdrone4/of_compare/imu_only_pose \
  use_speed_profile:=false \
  restore_speed_profile_on_exit:=false \
  quality_min:=20 \
  range_min_m:=0.3 \
  range_max_m:=4.5 \
  fusion_enabled:=true \
  imu_only_enabled:=true \
  fusion_max_sensor_age_s:=0.15 \
  fusion_max_flow_gap_s:=0.25
```

Before arming in QGC/operator setup:

- select `STABILIZED` before arming
- confirm RC/manual input is active and the kill/disarm switch is ready
- keep `EKF2_OF_CTRL=0`; optical flow, VIO, ROS fusion, and ZED images are
  logging-only
- confirm `/cdrone/cdrone4/mavros/vision_pose/pose` is mocap/external-vision
  input from `RigidBody4`, not VIO or optical-flow fusion

Monitor during the manual hover:

```bash
while true; do
  date -u +"[stabilized-hover-cdrone4-rb4] %H:%M:%S UTC"
  ros2 topic echo --once /cdrone/cdrone4/mavros/state 2>/dev/null | rg 'connected:|armed:|guided:|manual_input:|mode:|system_status:'
  ros2 topic echo --once /cdrone/cdrone4/mavros/local_position/pose 2>/dev/null | rg 'x:|y:|z:' | head -6
  ros2 topic echo --once /cdrone/cdrone4/mavros/vision_pose/pose 2>/dev/null | rg 'x:|y:|z:' | head -6
  ros2 topic echo --once /cdrone/cdrone4/external_pose/debug/summary 2>/dev/null | sed -n '1,12p'
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/state_raw.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/mocap_raw.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/px4_vision_pose_raw.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/px4_local_pose_raw.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/imu_raw.csv" 2>/dev/null
  find "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/images_dataset" -maxdepth 1 -name '*.jpg' 2>/dev/null | wc -l
  sleep 1
done
```

Abort/force-disarm are emergency actions only; for this manual test, the
operator normally lands and disarms from the RC/QGC controls. After disarm,
generate the diagnostics:

```bash
scripts/of_compare_stats.py "flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv" --plots
```

If the start service rejects with `global origin is missing`, publish a fresh
transient-local copy of the indoor origin, then start again:

```bash
ros2 topic pub --once \
  --qos-reliability reliable \
  --qos-durability transient_local \
  /cdrone/cdrone4/mavros/global_position/gp_origin \
  geographic_msgs/msg/GeoPointStamped \
  "{header: {frame_id: earth}, position: {latitude: 0.0, longitude: 0.0, altitude: 17.163000000000004}}"
```

After a fresh operator go/no-go, start:

```bash
ros2 service call /cdrone/cdrone4/demo/minjerk_waypoints_start std_srvs/srv/Trigger "{}"
```

Monitor state, arming, mode, local pose, external pose health, logs, and image
count:

```bash
while true; do
  date -u +"[minjerk-cdrone4-rb4] %H:%M:%S UTC"
  ros2 topic echo --once /cdrone/cdrone4/demo/minjerk_waypoints_state 2>/dev/null | sed -n '1,4p'
  ros2 topic echo --once /cdrone/cdrone4/mavros/state 2>/dev/null | rg 'connected:|armed:|mode:|system_status:'
  ros2 topic echo --once /cdrone/cdrone4/mavros/local_position/pose 2>/dev/null | rg 'x:|y:|z:' | head -6
  ros2 topic echo --once /cdrone/cdrone4/external_pose/debug/summary 2>/dev/null | sed -n '1,12p'
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/fusion_events.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/imu_raw.csv" 2>/dev/null
  tail -n 3 "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/optical_flow_raw.csv" 2>/dev/null
  find "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/images_dataset" -maxdepth 1 -name '*.jpg' 2>/dev/null | wc -l
  sleep 2
done
```

Abort immediately on operator command, stale external pose, FCU disconnect,
unexpected mode behavior, perimeter violation, uncontrolled drift, or a bad
takeoff. For this cdrone4 drift run, also abort if takeoff/hover forward drift
exceeds about `0.5 m` or position error grows steadily for more than `3 s`.

```bash
ros2 service call /cdrone/cdrone4/demo/minjerk_waypoints_abort std_srvs/srv/Trigger "{}"
```

After abort or landing, verify safe state, restored parameters, image capture,
and post-run stats:

```bash
ros2 topic echo --once /cdrone/cdrone4/mavros/state
ros2 topic echo --once /cdrone/cdrone4/mavros/local_position/pose

ros2 service call /cdrone/cdrone4/mavros/param/get_parameters \
  rcl_interfaces/srv/GetParameters \
  "{names: [MIS_TAKEOFF_ALT, MPC_XY_CRUISE, MPC_XY_VEL_MAX, MPC_ACC_HOR, MPC_JERK_AUTO, MPC_JERK_MAX, MPC_TKO_SPEED, MPC_Z_V_AUTO_UP, MPC_Z_V_AUTO_DN, MPC_Z_VEL_MAX_DN, MPC_LAND_SPEED]}"

wc -l "/home/jetson/cdrone_control/flight_logs/of_compare/${RUN_ID}/images_dataset/images.csv"
scripts/of_compare_stats.py "flight_logs/of_compare/${RUN_ID}/of_mocap_compare.csv" --plots
```

If PX4 remains in an undesired mode after the vehicle is safely disarmed, return
it to loiter:

```bash
ros2 service call /cdrone/cdrone4/mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.LOITER'}"
```

## Start

Run this only after the launch command above is still running, MAVROS is
connected, mocap is live, PX4Flow topics pass the checker, and the props-off
frame sanity check passes:

```bash
ros2 service call /cdrone/cdrone4/demo/minjerk_waypoints_start std_srvs/srv/Trigger "{}"
```

The mission will take off, switch to OFFBOARD, run the minimum-jerk waypoint
trajectory, hold, land, and restore any changed takeoff/speed params through
the existing cleanup path.

## Abort

```bash
ros2 service call /cdrone/cdrone4/demo/minjerk_waypoints_abort std_srvs/srv/Trigger "{}"
```

## Outputs

Primary CSV:

```text
flight_logs/of_compare/<run_id>/of_mocap_compare.csv
```

Raw callback events and resolved launch/runtime configuration are written to:

```text
flight_logs/of_compare/<run_id>/fusion_events.csv
flight_logs/of_compare/<run_id>/run_metadata.json
```

Variable-rate raw sensor logs are written at callback cadence, not at the
comparison CSV timer rate:

```text
flight_logs/of_compare/<run_id>/imu_raw.csv
flight_logs/of_compare/<run_id>/optical_flow_raw.csv
flight_logs/of_compare/<run_id>/state_raw.csv
flight_logs/of_compare/<run_id>/mocap_raw.csv
flight_logs/of_compare/<run_id>/px4_vision_pose_raw.csv
flight_logs/of_compare/<run_id>/px4_local_pose_raw.csv
```

Important columns:

- `mocap_frame_id`, `of_frame_id`, `setpoint_frame_id`, `frame_match`
- `origin_reset_mocap_x_m`, `origin_reset_mocap_y_m`, `origin_reset_mocap_z_m`
- `mocap_x_m`, `mocap_y_m`, `mocap_z_m`
- `of_x_m`, `of_y_m`, `of_z_m`
- `imu_only_x_m`, `imu_only_y_m`, `imu_only_z_m`,
  `imu_only_vx_mps`, `imu_only_vy_mps`, `imu_only_vz_mps`
- `imu_only_err_x_m`, `imu_only_err_y_m`, `imu_only_err_z_m`,
  `imu_only_err_xy_m`, `imu_only_err_3d_m`, `imu_only_pose_valid`
- `err_x_m`, `err_y_m`, `err_z_m`, `err_xy_m`, `err_3d_m`
- `imu_yaw_rad`, `imu_accel_x_mps2`, `imu_accel_y_mps2`,
  `imu_accel_z_mps2`
- `fused_x_m`, `fused_y_m`, `fused_z_m`
- `fused_err_x_m`, `fused_err_y_m`, `fused_err_z_m`,
  `fused_err_xy_m`, `fused_err_3d_m`
- `fusion_pose_valid`, `fusion_flow_valid`, `fusion_reject_reason`
- `fusion_imu_valid`, `fusion_imu_reject_reason`,
  `fusion_flow_valid`, `fusion_flow_reject_reason`
- `fusion_health_state`, `fusion_flow_nis`, `fusion_flow_gyro_source`,
  covariance, innovation, and effective measurement-variance columns
- source/receive timestamps, age, frame, flow sequence, inter-message interval,
  and integration-coverage columns
- `flow_quality`, `flow_distance_m`, `range_m`, `sample_valid`
- `px4_mode`, `px4_armed`, `px4_connected`, local pose columns,
  vision-pose-input columns, and local/vision-vs-mocap XY error columns
- raw sensor logs include `source_dt_s`, `receive_dt_s`, `source_rate_hz`, and
  `receive_rate_hz` so delivered IMU and optical-flow rates can vary sample by
  sample

At `COMPLETE`, the logger zeroes estimator velocities and freezes OF, IMU-only,
and fused poses so post-landing callbacks cannot inflate drift metrics.

Generate stats:

```bash
scripts/of_compare_stats.py flight_logs/of_compare/minjerk_of_001/of_mocap_compare.csv
```

The script writes:

```text
flight_logs/of_compare/<run_id>/report/summary.json
flight_logs/of_compare/<run_id>/report/summary.md
flight_logs/of_compare/<run_id>/report/drift_timeseries.csv
flight_logs/of_compare/<run_id>/report/fusion_timeseries.csv
flight_logs/of_compare/<run_id>/report/plots/flight_mode_over_time.png
flight_logs/of_compare/<run_id>/report/plots/px4_imu_health_over_time.png
flight_logs/of_compare/<run_id>/report/plots/mocap_health_over_time.png
flight_logs/of_compare/<run_id>/report/plots/px4_pose_vs_mocap_over_time.png
flight_logs/of_compare/<run_id>/report/plots/px4_vision_input_vs_mocap_over_time.png
```

The stats include separate X, Y, Z, horizontal XY, and 3D error summaries,
drift rates over time, rejection counts, flow range/quality health, IMU health,
and an identical-timestamp OF-only vs IMU-only vs IMU+OF comparison when those
columns are present.
If `valid_3d_rows` is zero, use the range/quality summary and
`reject_reasons` first; drift cannot be trusted until OF X/Y/Z rows are valid.
If `fusion.valid_3d_rows` is zero in `summary.json`, inspect
`fusion_reject_reasons`, `imu` health, and the `/mavros/imu/data` topic before
trusting the fused estimate.

The stats and plots are post-run steps; the flight launch does not call them
automatically after landing. Plots are off by default. Generate the required PNG
figures with:

```bash
scripts/of_compare_stats.py flight_logs/of_compare/minjerk_of_005/of_mocap_compare.csv --plots
```

For legacy recordings such as `minjerk_of_004`, run the timestamped three-way
EKF replay and blocked tuning search:

```bash
scripts/replay_imu_of_fusion.py \
  flight_logs/of_compare/minjerk_of_005/of_mocap_compare.csv --plots
```

It writes `replay/ekf_compare.csv`, `replay/tuning.json`,
`replay/replay_metadata.json`, and the replay report. Legacy IMU-only results
are explicitly marked approximate because the old main CSV contains 20 Hz
snapshots rather than every IMU callback. New runs should use
`fusion_events.csv` for callback-faithful replay.

## Troubleshooting

- `mocap frame ... does not match waypoint frame`: set the waypoint YAML
  `frame_id` to the mocap frame or fix the mocap launch frame.
- `local pose frame ... does not match waypoint frame`: check MAVROS
  `local_position.frame_id` and external-vision bridge frame settings.
- `waypoint ... is unsafe`: edit the waypoint or perimeter margin; the node
  checks each waypoint and each leg before arming.
- `sample_valid=0`: inspect `reject_reason`, `flow_quality`, and range columns.
- `fusion_pose_valid=0`: confirm the estimator received mocap at startup and
  the launch stack is publishing the configured IMU topic. For the cdrone4
  RigidBody4 run, that is `/cdrone/cdrone4/mavros/imu/data`.
- `fusion_flow_valid=0`: inspect `fusion_reject_reason`, `flow_quality`, and
  range columns, then inspect NIS, gyro source, sensor age, and integration
  coverage.
- Large XY drift with correct signs: reduce speed, improve lighting/texture, or
  increase `quality_min` before trusting the OF estimate. Confirm flow velocity
  has the same directional correlation as mocap before tuning the EKF.
- `gyro_integral_unavailable`: make the IMU topic and timestamps available over
  the full flow exposure, or repair the sensor-provided gyro integral stream.
- `flow_out_of_order` or stale measurements: repair time synchronization and
  delivery before increasing reorder/age limits.
- High `fusion_flow_nis`: verify frame/extrinsic yaw, scale/sign, range, and
  timing first; increase measurement noise only after those checks pass.
- Large IMU-only drift is expected from bias and vibration. It is a diagnostic
  baseline and must not be used to justify trusting the fused estimate.
