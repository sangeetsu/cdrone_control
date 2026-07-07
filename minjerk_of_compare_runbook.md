# Minimum-Jerk Mocap vs Optical-Flow Runbook

This runbook launches an additive test path:

- waypoint flight uses mocap/PX4 external-vision position for control
- host-side PX4Flow dead-reckon is observational
- mocap and optical-flow X, Y, and Z estimates are compared in the same `map`
  frame
- output CSV and stats are written under `flight_logs/of_compare/<run_id>/`

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Edit Waypoints

Edit:

```bash
/home/jetson/cdrone_control/ros2/src/drone_bringup/config/minjerk_waypoints.yaml
```

The file uses absolute `map` coordinates:

```yaml
waypoints:
  - name: east
    x_m: 0.75
    y_m: 0.00
    z_m: 1.20
    yaw_deg: 0.0
```

Keep `frame_id: map` unless the mocap system is intentionally publishing a
different frame. The mission node refuses to start if waypoint, mocap, local
pose, or perimeter frames disagree.

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
`/cdrone/cdrone3/of_compare/pose`. If only `/parameter_events` and `/rosout`
appear, the launch stack is not visible in that terminal.

Then verify the PX4Flow topics:

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
scripts/check_px4flow_topics.sh --request-stream-rate 10
```

With the launch still running, watch mocap and OF pose:

```bash
ros2 topic echo /cdrone/cdrone3/vrpn_mocap/RigidBody3/pose --once
ros2 topic echo /cdrone/cdrone3/of_compare/pose --once
```

Move the drone by hand in mocap +X, then +Y. The OF pose should change with
the same signs after the logger receives PX4Flow samples. If signs are wrong,
rerun with launch overrides until signs match:

```bash
scripts/run_minjerk_of_compare.sh --run-id flow_sign_test -- \
  flow_scale_x:=-1.0 \
  flow_scale_y:=1.0
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
  --run-id minjerk_of_002 \
  -- \
  quality_min:=20 \
  range_min_m:=0.3 \
  range_max_m:=4.5
```

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

Important columns:

- `mocap_frame_id`, `of_frame_id`, `setpoint_frame_id`, `frame_match`
- `origin_reset_mocap_x_m`, `origin_reset_mocap_y_m`, `origin_reset_mocap_z_m`
- `mocap_x_m`, `mocap_y_m`, `mocap_z_m`
- `of_x_m`, `of_y_m`, `of_z_m`
- `err_x_m`, `err_y_m`, `err_z_m`, `err_xy_m`, `err_3d_m`
- `flow_quality`, `flow_distance_m`, `range_m`, `sample_valid`

Generate stats:

```bash
scripts/of_compare_stats.py flight_logs/of_compare/minjerk_of_001/of_mocap_compare.csv
```

The script writes:

```text
flight_logs/of_compare/<run_id>/report/summary.json
flight_logs/of_compare/<run_id>/report/summary.md
flight_logs/of_compare/<run_id>/report/drift_timeseries.csv
```

The stats include separate X, Y, Z, horizontal XY, and 3D error summaries,
drift rates over time, rejection counts, and flow range/quality health.
If `valid_3d_rows` is zero, use the range/quality summary and
`reject_reasons` first; drift cannot be trusted until OF X/Y/Z rows are valid.

Plots are off by default. Generate optional PNG plots only when you explicitly
ask for them:

```bash
scripts/of_compare_stats.py flight_logs/of_compare/minjerk_of_001/of_mocap_compare.csv --plots
```

## Troubleshooting

- `mocap frame ... does not match waypoint frame`: set the waypoint YAML
  `frame_id` to the mocap frame or fix the mocap launch frame.
- `local pose frame ... does not match waypoint frame`: check MAVROS
  `local_position.frame_id` and external-vision bridge frame settings.
- `waypoint ... is unsafe`: edit the waypoint or perimeter margin; the node
  checks each waypoint and each leg before arming.
- `sample_valid=0`: inspect `reject_reason`, `flow_quality`, and range columns.
- Large XY drift with correct signs: reduce speed, improve lighting/texture, or
  increase `quality_min` before trusting the OF estimate.
