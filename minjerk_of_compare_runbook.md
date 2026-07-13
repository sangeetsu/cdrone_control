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
  --run-id minjerk_of_005 \
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

## Start

Run this only after the launch command above is still running, MAVROS is
connected, mocap is live, PX4Flow topics pass the checker, and the props-off
frame sanity check passes:

```bash
ros2 service call /cdrone/cdrone3/demo/minjerk_waypoints_start std_srvs/srv/Trigger "{}"
```

The mission will take off, switch to OFFBOARD, run the minimum-jerk waypoint
trajectory, hold, land, and restore any changed takeoff/speed params through
the existing cleanup path.

## Abort

```bash
ros2 service call /cdrone/cdrone3/demo/minjerk_waypoints_abort std_srvs/srv/Trigger "{}"
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
  the launch stack is publishing `/cdrone/cdrone3/mavros/imu/data`.
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
