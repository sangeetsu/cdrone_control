# Autonomous Optical-Flow Hover Runbook

This runbook brings up autonomous hover without mocap. Position estimates come
from PX4Flow optical flow, the PX4Flow range output, and MAVROS IMU data. The
ROS estimator publishes a fused `PoseStamped` into the same external-pose bridge
contract used by the existing hover demo.

## Hardware Assumptions

- PX4Flow or ARK/PX4-compatible optical-flow sensor is connected to PX4.
- MAVROS publishes:
  - `/cdrone/<drone_id>/mavros/px4flow/raw/optical_flow_rad`
  - `/cdrone/<drone_id>/mavros/px4flow/ground_distance`
  - `/cdrone/<drone_id>/mavros/imu/data`
- No OptiTrack, VRPN, mocap rigid body, RealSense VIO, or external camera pose is
  required for this hover path.
- Use a textured, non-glossy floor with enough lighting for optical flow.
- Start with props off for bench checks; only move to props on after the
  estimator signs, scale, and freshness gates are sane.

## Preflight Checks

Source ROS and this workspace:

```bash
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
```

Confirm MAVROS is connected:

```bash
ros2 topic echo --once /cdrone/cdrone3/mavros/state
```

Confirm PX4Flow streams are present. This can also request the native-rate flow
stream:

```bash
scripts/check_px4flow_topics.sh \
  --mavros-namespace /cdrone/cdrone3/mavros \
  --request-stream-rate 70 \
  --duration 10
```

Confirm the IMU is live:

```bash
ros2 topic hz /cdrone/cdrone3/mavros/imu/data
```

Start the mocap-free hover stack, but do not start the flight yet:

```bash
ros2 launch drone_bringup of_position_hover_demo.launch.py \
  drone_id:=cdrone3 \
  mavros_namespace:=/cdrone/cdrone3/mavros \
  drone_namespace:=/cdrone/cdrone3 \
  takeoff_altitude_m:=0.5 \
  hover_duration_s:=3.0 \
  max_horizontal_excursion_m:=0.5 \
  allow_synthetic_home_position:=true \
  synthetic_home_position_z_m:=0.0
```

Confirm estimator, bridge, and PX4-facing vision pose:

```bash
ros2 topic echo --once /cdrone/cdrone3/of_hover/status
ros2 topic echo --once /cdrone/cdrone3/of_hover/fused_pose
ros2 topic echo --once /cdrone/cdrone3/external_pose/input_pose
ros2 topic echo --once /cdrone/cdrone3/mavros/vision_pose/pose
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
```

Expected status: `"ready": true`. If the external pose topic does not publish,
read the `reason` field in `/cdrone/cdrone3/of_hover/status`.

## Parameter And Profile Checklist

Before flight, verify PX4 is configured to accept the chosen external/local
position path for indoor hover. Keep the current hover baseline unless you are
intentionally tuning:

```bash
scripts/restore_hover_baseline_params.sh
```

Recommended first-pass launch parameters:

- `quality_min:=10`
- `range_min_m:=0.05`
- `range_max_m:=5.0`
- `allow_near_ground_flow:=true`
- `near_ground_range_m:=0.05`
- `allow_missing_flow_gyro:=true`
- `fusion_max_sensor_age_s:=0.25`
- `max_imu_age_s:=0.25`
- `max_range_age_s:=0.25`
- `max_flow_age_s:=0.50`
- `fusion_flow_sensor_yaw_rad:=0.0`
- `allow_synthetic_home_position:=true`
- `synthetic_home_position_z_m:=0.0`
- `require_global_origin:=false`
- `require_companion_active:=false`

If XY motion is sign-flipped or rotated during hand tests, tune:

- `flow_scale_x`
- `flow_scale_y`
- `fusion_flow_sensor_yaw_rad`

Keep first flights slow and low. Enable the indoor speed profile only after the
bench checks are clean:

```bash
ros2 launch drone_bringup of_position_hover_demo.launch.py \
  use_speed_profile:=true \
  speed_profile_config:=/home/jetson/cdrone_control/ros2/src/drone_bringup/config/indoor_speed_profile.yaml
```

## Ground Test

Run the launch with props off.

1. Hold the drone still on the floor. The status topic should become ready.
2. Lift it slightly and confirm `z` follows the range sensor.
3. Move the drone forward/back/left/right by hand and inspect:

```bash
ros2 topic echo /cdrone/cdrone3/of_hover/fused_pose
```

4. Confirm the reported XY direction matches the map/body convention expected by
   PX4 local position.
5. Yaw the drone by hand and confirm yaw changes smoothly without large XY jumps.
6. Cover or starve the flow sensor briefly. The status reason should move toward
   `flow_quality_below_min`, `flow_stale`, or a fusion reject reason, and
   `/external_pose/input_pose` should stop publishing until health recovers.

## First Flight

Use the most conservative settings first:

```bash
ros2 launch drone_bringup of_position_hover_demo.launch.py \
  drone_id:=cdrone3 \
  mavros_namespace:=/cdrone/cdrone3/mavros \
  drone_namespace:=/cdrone/cdrone3 \
  takeoff_altitude_m:=0.5 \
  hover_duration_s:=3.0 \
  hover_mode:=HOLD \
  max_horizontal_excursion_m:=0.5 \
  enable_pose_debug:=true
```

In another terminal, start the hover sequence only after the estimator is ready:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_start std_srvs/srv/Trigger {}
```

Abort immediately if XY drift grows, range becomes unstable, or local position
stops updating:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_abort std_srvs/srv/Trigger {}
```

Monitor during flight:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_hover_state
ros2 topic echo /cdrone/cdrone3/of_hover/status
ros2 topic echo /cdrone/cdrone3/mavros/local_position/pose
```

## Troubleshooting

- No PX4Flow topics: run `scripts/check_px4flow_topics.sh --request-stream-rate 70`
  and confirm MAVROS extras include the `px4flow` plugin.
- Status `imu_stale`: confirm `/mavros/imu/data` rate and that this terminal has
  the same `ROS_DOMAIN_ID` as MAVROS.
- Status `range_stale`, `range_invalid`, or `range_below_min`: inspect
  `/mavros/px4flow/ground_distance`. This launch defaults
  `allow_near_ground_flow:=true`, `range_min_m:=0.05`, and
  `near_ground_range_m:=0.05`, so too-close finite ground readings are clamped
  for optical-flow scaling. In `/of_hover/status`, compare raw `range_m` against
  `effective_flow_range_m`.
- Status `flow_quality_below_min`: improve floor texture/lighting or lower
  `quality_min` only for controlled tests.
- Status `flow_stale`: request the flow stream rate and inspect
  `/mavros/px4flow/raw/optical_flow_rad`.
- Fusion reject `flow_state_skew`, `flow_gap_exceeded`, or `sensor_stale`: check
  timestamps, stream rates, and `fusion_reorder_tolerance_s`.
- Fusion reject `gyro_integral_unavailable`: this launch defaults
  `allow_missing_flow_gyro:=true`, which substitutes zero gyro integral when
  PX4Flow gyro fields are missing. Keep yaw motions small during first hover
  tests when this fallback is active.
- XY sign or yaw inversion: tune `flow_scale_x`, `flow_scale_y`, and
  `fusion_flow_sensor_yaw_rad` with props off.
- PX4 local pose not updating: confirm `/external_pose/input_pose` and
  `/mavros/vision_pose/pose` publish while status is ready, then verify PX4
  estimator parameters for external-vision/local-position use.
- `external_pose_debug_node` reports a large adapter-to-local vertical mismatch:
  PX4/MAVROS local Z is not aligned with the optical-flow estimator. The
  optical-flow hover launch feeds `/cdrone/cdrone3/of_hover/fused_pose` to the
  hover demo as `demo_local_pose_topic`, so the demo's altitude checks follow
  optical flow instead of drifting MAVROS local Z. Treat the warning as a PX4
  local-estimator alignment issue to solve before faster flights.
- Demo start says `home position is missing`: this mocap-free launch defaults
  `allow_synthetic_home_position:=true`. If it still appears, confirm the
  launch argument was not overridden and that `/mavros/local_position/pose` is
  fresh. For strict MAVROS home debugging, set
  `allow_synthetic_home_position:=false` and inspect
  `/mavros/home_position/home`.
- Demo start says `global origin is missing`: this mocap-free launch defaults
  `require_global_origin:=false`. If it still appears, relaunch without
  overriding that argument and re-source the rebuilt install workspace.
- Demo start says `external pose source is not active`: this mocap-free launch
  defaults `require_companion_active:=false` because optical flow is the pose
  source for this workflow. If you still see the message, relaunch without
  overriding that argument and confirm the installed workspace was re-sourced.
- Horizontal excursion abort: reduce `takeoff_altitude_m`, shorten
  `hover_duration_s`, increase floor texture, and inspect estimator status before
  loosening `max_horizontal_excursion_m`.
