# Milestone 1 Runbook

## Onboard Localization: Start Here

The first onboard test is a local hover using **ARK Flow over CAN + PX4's IMU,
barometer and calibrated heading source**. PX4 performs sensor fusion; the
Jetson sends local position setpoints through MAVROS, with an optional initial
attitude-and-thrust climb when flow cannot operate at ground clearance. No camera-based odometry
implementation is needed for this step.

The hover launch now has two paths:

| Argument | Behavior |
| --- | --- |
| `localization_source:=onboard` | MAVROS + local OFFBOARD hover; no mocap, pose injection, fixed global origin or home setup |
| `localization_source:=external_pose` | Original mocap/external-pose hover; compatibility default |
| `enable_flight:=false` | Onboard diagnostics only; no setpoints, arming or mode commands; default |
| `enable_flight:=true` | Onboard start service may command takeoff/hover/landing after readiness checks |
| `initial_climb_enabled:=true` | Onboard: climb using downward range and PX4 attitude control, then acquire position hold; default false |

`enable_flight` applies only to the onboard path. It does **not** make the legacy
external-pose demo a bench-only program.

The code/instructions update does not apply FCU parameters, reboot the FCU or
perform a flight. Keep the ESCs disconnected for the configuration/readiness
stage. Diagnostic success on the bench is not proof of flight stability.

### 1. Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

### 2. Inspect And Back Up FCU Settings

The inspected FCU runs PX4 **1.16.0**. During the initial September 13 inspection,
`UAVCAN_SUB_FLOW=0` and `UAVCAN_SUB_RNG=0`; PX4's internal
`sensor_optical_flow` and `distance_sensor` topics had never published that boot.
External vision fusion was already off. A local-position message was present,
but the estimator had no valid horizontal position/velocity. These were
configuration blockers at that time, not a proven explanation of the earlier offset.

The following inspector connects through the existing mavlink-router; it does
not require the hover launch or MAVROS. It only requests parameters and telemetry.
It understands PX4's bytewise integer parameter encoding.

```bash
cd /home/jetson/cdrone_control
python3 scripts/inspect_onboard_localization.py --backup backup/onboard_before.yaml
```

When MAVROS is already running, this Jetson's dedicated `companion_aux` router
endpoint avoids competing for the shared UDP server's return address:

```bash
python3 scripts/inspect_onboard_localization.py --port udpin:127.0.0.1:14569
```

Use that endpoint for one inspection client at a time.

Exit status `2` means settings differ or firmware is outside the profile's PX4
1.16 scope; `1` means inspection failed. The backup contains the original values
of every parameter the profile would change. Existing backup files are never
overwritten: choose a new filename for a new session. Also export a full FCU
parameter backup from QGroundControl before applying changes.

Review [the onboard parameter profile](ros2/src/drone_bringup/config/onboard_optical_flow_profile.yaml).
It enables CAN flow/range subscriptions, uses barometric height reference with
conditional range aiding, disables external vision/GNSS fusion, and sets
OFFBOARD-loss action to **Land** after one second. Sensor limits match the
standard ARK Flow, not ARK Flow MR. Calibration, mounting offsets and controller
gains are retained.

Verify the actual downward sensor orientation and measured sensor offsets. The
inspection found `SENS_FLOW_ROT=0` and `EKF2_OF_POS_X=0.06 m`; retaining those
values does not verify their physical correctness. A calibrated heading source
is still required when external-vision yaw is disabled.

### 3. Operator Applies The Reviewed Configuration

Print the exact console commands **without connecting to the FCU**:

```bash
python3 scripts/inspect_onboard_localization.py --print-commands
```

With the FCU disarmed, apply the reviewed values in QGroundControl's parameter
editor, or enter the printed `param set ...` lines and `param save` in its MAVLink
console. Reboot the FCU using QGroundControl, then inspect again:

```bash
python3 scripts/inspect_onboard_localization.py
```

Do not use `restore_hover_baseline_params.sh` for this onboard setup: that legacy
profile also overwrites accelerometer calibration. The hover node never applies
profiles or changes `MIS_TAKEOFF_ALT` automatically.

To roll back, print commands from the saved original values, review/apply them
while disarmed, save, reboot, and verify:

```bash
python3 scripts/inspect_onboard_localization.py \
  --profile backup/onboard_before.yaml --print-commands
```

### 4. Confirm Data Inside PX4

In QGroundControl's MAVLink console, these read-only commands distinguish sensor
problems from ROS/telemetry problems:

```text
uavcan status
listener sensor_optical_flow -n 1
listener distance_sensor -n 1
listener estimator_status_flags -n 1
listener vehicle_local_position -n 1
```

Confirm fresh flow/range samples, downward distance orientation, flow quality,
and `cs_opt_flow: True` when the sensor has a usable surface/range. Confirm valid
horizontal velocity and position; `cs_inertial_dead_reckoning: True` with no flow
fusion is not hover readiness. Flow needs a lit, textured stationary floor and
usable range; placing the sensor too close to the floor or obstructing it can
prevent readiness. Resolve these conditions rather than bypassing estimator checks.

ROS topics require MAVROS running. PX4's internal sensor topics do not depend on
the hover launch. Missing MAVLink samples alone can also mean the stream is not
enabled, which is why both checks are provided.

### 5. ESC-Disconnected Readiness Test

Stop other demo/control launches first. In a sourced terminal:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=onboard enable_flight:=false
```

The onboard overlay denies mocap/vision/fake-GPS injection plugins. The hover node
uses `/cdrone/cdrone3/mavros/px4flow/ground_distance`, which MAVROS publishes from
the distance field of PX4's `OPTICAL_FLOW_RAD` stream. The overlay sets this
wrapper's acceptance limits to **0.05–30 m**, using the operator-requested 5 cm
minimum for ground readings; MAVROS otherwise
uses legacy PX4Flow sonar limits of 0.3–5 m. Its `radiation_type` still says
ULTRASOUND because MAVROS hardcodes that field; it does not identify ARK hardware.
These are ROS metadata settings, not FCU parameter changes.
The [documented ARK optical-flow working height](https://docs.px4.io/v1.16/en/dronecan/ark_flow)
starts at 8 cm. The FCU profile retains its documented 8 cm flow/range settings;
accepting a closer distance in ROS does not establish usable optical-flow fusion.

The optional `DISTANCE_SENSOR` mapping for downward ID `0` is retained. To use
it, first verify that `/cdrone/cdrone3/mavros/distance_sensor/ark_flow` actually
publishes the intended sensor, then pass that full path as `range_topic:=...`.
The `flow_topic` launch argument can also override the default flow stream.
Do not substitute an unrelated range sensor. The node requests telemetry rates for local position,
flow, distance, estimator and landed state; these requests do not alter EKF or
sensor parameters.

In another sourced terminal:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_check std_srvs/srv/Trigger "{}"
ros2 topic echo /cdrone/cdrone3/demo/position_hover_diagnostics --full-length
```

Useful topics to inspect separately:

```bash
ros2 topic echo --once /cdrone/cdrone3/mavros/state
ros2 topic echo --once /cdrone/cdrone3/mavros/extended_state
ros2 topic echo --once /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 topic echo --once /cdrone/cdrone3/mavros/px4flow/raw/optical_flow_rad
ros2 topic echo --once /cdrone/cdrone3/mavros/px4flow/ground_distance
```

`position_hover_check` reports readiness separately from `enable_flight`. It
requires two continuous seconds of valid data, ON_GROUND and disarmed state.
Sensor/pose freshness is 0.5 seconds; state/estimator freshness is 2.5 seconds.
Stale, repeated, out-of-order or non-finite pose/pixel-flow/range data do not count
as valid. PX4 must report valid attitude, horizontal position/velocity and
vertical estimates, plus active horizontal aiding. With the original CAN subscriptions disabled,
the expected result is **not ready**, with flow/range and estimator blockers.

Troubleshooting the range/flow diagnostics:

- `range: stale or missing`: check the reported `range_topic` and its timestamps.
  Restart the hover launch after rebuilding an update; an existing process keeps
  its old subscriptions and MAVROS parameter overlay.
- `range: distance=0.049 m outside usable [0.050, 30.000] m`: the reading is below
  the configured 5 cm minimum. A reading of `0.075 m` now passes the range check
  on the default topic. PX4 must still report active horizontal aiding and valid
  estimates before the sequence can start.
- A reported usable minimum of `0.080` or `0.300` on the default topic after
  upgrading indicates an older node or MAVROS wrapper configuration is running.
  Restart the complete launch, including MAVROS, so both changes take effect.
  An alternate range topic can still impose its own stricter message limits.
- `flow: quality=...` or `integration_time_us=...`: resolve the stated quality,
  integration or finite-pixel-flow issue. Optional integrated gyro fields may be
  NaN; they do not invalidate otherwise usable pixel flow because this node uses
  PX4's fused estimate rather than doing gyro compensation itself.
- `estimator: horizontal aiding inactive`: sensor packets are not proof that the
  EKF is using them. Check `cs_opt_flow` in `estimator_status_flags`, usable range,
  surface texture/lighting and the reviewed FCU profile. In PX4 1.16,
  `pred_pos_horiz_rel_status_flag` reports active horizontal aiding. The separate
  `const_pos_mode_status_flag` also becomes true when the vehicle is at rest, so
  it is informational here rather than a standalone failure. Missing aiding or
  invalid position/velocity still blocks starting and aborts an active sequence.

If flow/range pass but horizontal aiding remains inactive, inspect attitude as
well as flow inside PX4:

```text
listener vehicle_attitude -n 1
listener estimator_status_flags -n 1
listener estimator_aid_src_optical_flow -n 1
```

An upright, level airframe must show roll/pitch near zero. During the September 13
bench investigation, PX4 reported roll approximately **179 degrees** while the
operator confirmed the airframe was upright. All three accelerometers reported
positive Z near gravity, `SENS_BOARD_ROT=0`, `cs_opt_flow=False`, and optical-flow
`time_last_fuse=0`. This identifies an attitude/orientation problem upstream of
ROS. PX4's [flow fusion startup check](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/ekf2/EKF/aid_sources/optical_flow/optical_flow_control.cpp#L79)
rejects that tilt even when flow quality is good.

Verify the FCU mounting from its top side/forward arrow or controlled IMU
measurements before selecting its
[Autopilot Orientation in QGroundControl](https://docs.px4.io/v1.16/en/config/flight_controller_orientation).
The normal mounting assumption is top side up and arrow toward the nose. If the
FCU is mounted differently, the operator must configure the matching orientation
and perform the [required accelerometer calibration](https://docs.px4.io/v1.16/en/config/accelerometer).
Confirm the horizon follows actual airframe movements and then recheck flow
fusion. Do not compensate for an inverted FCU estimate by rotating the ROS pose
or disabling the horizontal-aiding check. A range reading near the flow minimum
may remain a separate limitation once attitude is corrected.

**cdrone3 mounting measurement — September 15, 2026:** the ARKV6X is hidden under
the ARK Jetson PAB stack. The operator confirmed the airframe was upright and
performed separate nose-up and right-side-down holds. With `SENS_BOARD_ROT=0`,
the measured `sensor_combined` acceleration vectors were:

| Airframe position | FCU X (m/s²) | FCU Y (m/s²) | FCU Z (m/s²) |
| --- | ---: | ---: | ---: |
| Level | -0.128 | -0.137 | +9.586 |
| Nose up | -0.438 | +2.324 | +9.322 |
| Right side down | -4.311 | +0.021 | +8.559 |

These measurements support **`SENS_BOARD_ROT=10` (Roll 180°, Yaw 90°)** for this
airframe. PX4's [rotation definition](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/conversion/rotation.h#L197)
maps the FCU vector `(X, Y, Z)` to body `(Y, X, -Z)`. Applied offline, this gives
approximately level roll/pitch for the first sample, positive pitch for nose-up,
and positive roll for right-side-down. This is a mounting inference from the
three holds, not a calibration or a post-change flight validation. It is specific
to this installation and is deliberately absent from the generic ARK Flow profile.

Operator correction, with ESCs disconnected and the FCU disarmed:

1. Export a full parameter backup in QGroundControl. The inspected original
   `SENS_BOARD_ROT` value is `0`.
2. Set **Autopilot Orientation → Roll 180°, Yaw 90°**, or set `SENS_BOARD_ROT` to
   `10` in the parameter editor. Equivalent MAVLink-console commands are:

   ```text
   param set SENS_BOARD_ROT 10
   param save
   ```

3. Reboot the FCU through QGroundControl. Complete its accelerometer calibration
   with the new orientation, then level-horizon calibration on a level surface.
4. Verify level roll/pitch are near zero, nose-up gives positive pitch, and
   right-side-down gives positive roll. Check that the displayed attitude follows
   physical motion before any arming or flight test.
5. Restart the onboard diagnostics launch with `enable_flight:=false`. Verify
   `cs_opt_flow=True`, a nonzero/recent optical-flow `time_last_fuse`, and successful
   `position_hover_check`. If fusion remains inactive, inspect its remaining
   range/quality/terrain conditions instead of assuming this correction proved readiness.

No orientation parameters, calibration values or FCU reboot were applied during
this inspection. `rpy_offset_rad` is an external-pose adapter setting and does not
correct the FCU attitude in onboard mode. The separately mounted ARK Flow sensor's
orientation must also match the airframe; do not copy this FCU rotation value into
`SENS_FLOW_ROT`.

The flag interpretation follows [PX4 1.16's estimator status implementation](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/modules/ekf2/EKF/ekf_helper.cpp#L618).
The range wrapper and optional gyro conversion are described by the
[MAVROS PX4Flow plugin implementation](https://github.com/mavlink/mavros/blob/ros2/mavros_extras/src/plugins/px4flow.cpp).

The start service must reject a start while `enable_flight:=false`. No home,
global-origin or companion external-pose heartbeat is required by this path.
Local Z need not be zero on the ground: the climb is relative to the pose
captured at start, in the same local ENU frame. Large absolute X/Y values alone
do not mean the drone will fly toward the origin.

### 6. Separate Operator Flight Test

Only after configuration, sensor orientation and bench readiness are verified,
stop the diagnostics launch and prepare the airframe for a supervised flight.
Use the physical flight-test setup and operator takeover procedure. This is a
separate physical test, not part of the ESC-disconnected validation.

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=onboard enable_flight:=true \
  takeoff_altitude_m:=1.0 takeoff_rate_m_s:=0.3 hover_duration_s:=20.0 \
  max_horizontal_excursion_m:=0.5
```

Check readiness, then start and monitor:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_check std_srvs/srv/Trigger "{}"
ros2 service call /cdrone/cdrone3/demo/position_hover_start std_srvs/srv/Trigger "{}"
ros2 topic echo /cdrone/cdrone3/demo/position_hover_state
```

Abort service:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

Expected progression: WARMUP (20 Hz for 1.5 s), SET_OFFBOARD, ARMING, TAKEOFF,
HOVER, LANDING, COMPLETE. The hover timer begins after altitude is within 0.15 m
of the requested height for one second. X/Y and yaw remain anchored to start.
Landing uses PX4 AUTO.LAND; normal disarm is requested only after fresh landed
confirmation persists for one second. No forced disarm or prearm-check bypass
is used.

Estimator loss, stale sensor data, a pose/frame jump, excessive excursion, leaving
the hover altitude band by more than 0.3 m, or no climb causes an abort/handoff.
Position setpoints stop on abort. If commands or
landing cannot be confirmed, the node reports FAILSAFE and requires operator
verification rather than claiming completion. Leaving OFFBOARD through the
operator's mode switch releases control; the node does not fight the new mode.
Stopping the node removes its setpoint stream and relies on the configured PX4
OFFBOARD-loss behavior; it does not automatically disarm in the air.

Acceptance: complete takeoff, the full hover interval, landing and disarm without
estimator faults, visible unstable motion, or excursion beyond the configured
limits. Record actual drift and altitude behavior; the software's estimated
position checks alone cannot measure drift of the estimator itself. Only after
this passes consider the earlier 1.5 m / 30 s hover settings.

### 7. Autonomous Initial Climb At Low Ground Clearance

Use this opt-in path when the ARK Flow lens is below its 8 cm operating height
on the ground. It adds `INITIAL_CLIMB` and `POSITION_HANDOVER` between ARMING and
TAKEOFF. It does not lower PX4's optical-flow limits or bypass PX4 arming checks.

The initial phase sends level-attitude/yaw and normalized thrust targets through
`setpoint_raw/attitude`; PX4 runs its attitude/rate controllers. A short vertical
loop uses fresh downward range, corrected for tilt using the independent IMU
orientation. Horizontal estimator validity is not required during this phase.
Fresh IMU, vertical estimator health, finite local pose/frame, raw flow packets
and valid range are still required. Warmup and arming send zero-thrust attitude
targets; increasing thrust starts only after armed telemetry is confirmed.

There is no horizontal position hold or reliable horizontal excursion measurement
before aiding is acquired. The first supervised validation needs room for lateral
drift. Do not treat the existing 0.5 m position perimeter as protection during
this short phase. The implementation has offline controller/ROS transport tests;
it has not been flight validated.

Start with diagnostics (no setpoints or flight commands):

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=onboard enable_flight:=false \
  initial_climb_enabled:=true initial_climb_hover_thrust:=0.62
```

`initial_climb_hover_thrust` must be supplied explicitly. `0.62` is the
`MPC_THR_HOVER` value read from cdrone3 on September 16; it is an existing airframe
setting, not a newly measured hover thrust. Verify it for the installed airframe
and propulsion configuration. The code does not read or change that FCU parameter.
The default `0.0` blocks readiness/start rather than guessing a thrust value.

For the separate supervised flight test, stop diagnostics and launch:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=onboard enable_flight:=true \
  initial_climb_enabled:=true initial_climb_hover_thrust:=0.62 \
  takeoff_altitude_m:=1.0 takeoff_rate_m_s:=0.3 hover_duration_s:=20.0
```

Use the same `position_hover_check`, `position_hover_start`, and
`position_hover_abort` services above. Enabling flight does not start the mission.

| Initial climb setting | Default / behavior |
| --- | --- |
| `initial_climb_height_m` | 0.25 m above measured starting range |
| `initial_climb_rate_m_s` | 0.2 m/s vertical speed limit |
| `initial_climb_max_thrust` | 0.75 normalized collective thrust ceiling |
| `initial_climb_thrust_slew` | Thrust increases by at most 0.25 per second |
| `initial_climb_min_range_m` | 0.15 m minimum tilt-corrected range for handover |
| `initial_climb_aiding_dwell_s` | 1 s continuous valid aiding and settled height |
| `initial_climb_no_lift_timeout_s` | Land if measured climb remains below 3 cm after 5 s |
| `initial_climb_timeout_s` | Land if handover is not ready within 8 s |
| `initial_climb_max_tilt_rad` | 0.2 rad (11.5 degrees) measured tilt limit |
| `initial_climb_max_speed_m_s` | 0.6 m/s measured vertical speed abort limit |

Handover requires range-confirmed climb within 4 cm of the initial target,
vertical speed no greater than 0.12 m/s, PX4 reporting IN_AIR, all normal position
and aiding checks, and fresh estimator updates throughout the dwell. PX4's
takeoff/land-detector flag alone does not establish liftoff. Local-Z drift alone
also cannot satisfy this gate.

After aiding settles, X/Y are anchored to the current valid pose. The final
altitude remains relative to the launch surface using the measured range climb.
Only position targets are sent during `POSITION_HANDOVER`; MAVROS must report a
fresh matching PX4 target on `setpoint_raw/target_local` within two seconds.
The launch requests MAVLink message 85 at 10 Hz for this confirmation.
Missing/mismatching echoes abort with `PX4 position handover not confirmed`.
The existing MAVROS `setpoint_raw.thrust_scaling=1.0` must be retained.

At no time are attitude and position commands streamed together. Once position
control starts, losing aiding invokes the existing landing path; the controller
never falls back to the unaided climb in flight. Excessive tilt, climb exceeding
the initial target by 15 cm, stale/invalid required data, unexpected yaw change,
no lift, or timeout stop initial-climb targets and request AUTO.LAND. An operator
mode change releases control. No forced disarm or automatic restart is used.

The default path with `initial_climb_enabled:=false` retains its original
requirement for horizontal aiding before starting.

The old room-coordinate goto/circle tests below are **mocap-only**. Optical flow
does not establish the studio map, pillar location or absolute room boundaries.

References: [ARK Flow setup](https://docs.px4.io/v1.16/en/dronecan/ark_flow),
[PX4 OFFBOARD control](https://docs.px4.io/v1.16/en/flight_modes/offboard),
[PX4 estimator parameters](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference).

---

## Legacy Mocap Test Sequence

Everything in the remaining sections assumes the external-pose path and the
studio's calibrated mocap frame. Restore the appropriate mocap EKF configuration
before using it; selecting the ROS launch alone does not configure PX4 fusion.

This runbook is for the first flight-test pass of:

- `position_hover_demo`
- `position_goto_demo`
- `position_circle_demo`

The active identity values come from
`ros2/src/drone_bringup/config/droneid_config.yaml`. The examples below reflect
the current `cdrone3` branch defaults.

The studio geometry plot for these coordinates is:

- `docs/generated/drone_studio_perimeter_2d.svg`

The intended sequence is:

1. prove hover still works
2. prove a short safe goto south of the pillar
3. prove a longer goto to the south-side staging point for the pillar orbit
4. only then try the circle demo

## Assumptions

- Motive is running and the active world frame is now `z-up`
- the drone is using the same rigid body as the hover demo path
- the perimeter file is:
  - `ros2/src/drone_bringup/config/drone_studio_perimeter.yaml`
- you are flying with the current local `map` frame from `/cdrone/cdrone3/mavros/local_position/pose`

If your active flight rigid body is not the default one, add:

```bash
rigid_body_name:=RigidBody3
```

or replace `RigidBody3` with the correct body name in the launch commands below.

## One-Time Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Terminal Setup

Use three terminals.

Terminal 1: launch the active demo

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

Terminal 2: watch demo state

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

Terminal 3: send start and abort service calls

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Preflight Checks

Run these before each flight attempt.

```bash
ros2 topic echo --once /cdrone/cdrone3/mavros/state
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 topic echo --once /cdrone/cdrone3/mavros/companion_process/status
ros2 topic hz /cdrone/cdrone3/mavros/local_position/pose
```

What you want to see:

- `/cdrone/cdrone3/mavros/state` is updating and connected
- `/cdrone/cdrone3/mavros/local_position/pose` is updating cleanly
- companion status is present
- local pose rate is healthy

Before any hover or goto test, restore the validated FCU baseline:

```bash
/home/jetson/cdrone_control/scripts/restore_hover_baseline_params.sh
```

Do not enable the indoor speed profile for now. On the current FCU/hardware combination, `use_speed_profile:=true` can stall takeoff even though arming succeeds. If a run was started with `use_speed_profile:=true` or was interrupted during parameter restore, rerun the baseline restore script before the next flight.

## Phase 1: Hover Demo

Terminal 1:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=external_pose \
  takeoff_altitude_m:=1.5 \
  hover_duration_s:=30.0 \
  use_speed_profile:=false \
  rpy_offset_rad:='[0.0, 0.0, -3.141592653589793]'
```

Terminal 2:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_hover_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

Expected result:

- arm
- take off
- hover briefly
- land
- disarm

If hover is not clean, stop here and do not continue to goto.

## Phase 2: Short Goto South Of The Pillar

Use this only if the drone is starting near the southwest side of the room. This goal is a short confidence hop at `(-11.5, -2.5)` and stays well south of the pillar.

Terminal 1:

```bash
ros2 launch drone_bringup position_goto_demo.launch.py \
  goal_x_m:=-11.5 \
  goal_y_m:=-2.5 \
  goal_z_m:=2.0 \
  max_goal_distance_from_start_m:=3.0 \
  use_speed_profile:=false \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_abort std_srvs/srv/Trigger "{}"
```

Expected result:

- take off
- switch to OFFBOARD
- move to the short south-of-pillar goal
- hold
- land

## Phase 3: Longer Goto To The Circle Staging Point

Use the south-side staging point for the pillar-centered orbit:

- `goal_x_m:=-5.62`
- `goal_y_m:=0.40`

This point is south of the pillar keep-out, stays comfortably inside the studio boundary, and lines up with the current circle demo geometry. Keep the baseline FCU parameters in place for this phase.

Terminal 1:

```bash
ros2 launch drone_bringup position_goto_demo.launch.py \
  goal_x_m:=-5.62 \
  goal_y_m:=0.40 \
  goal_z_m:=2.0 \
  max_goal_distance_from_start_m:=12.0 \
  use_speed_profile:=false \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_abort std_srvs/srv/Trigger "{}"
```

Expected result:

- take off
- switch to OFFBOARD
- fly to the south-side staging point at `2.0 m`
- hold
- land near that point

## Phase 4: Circle Demo

Only do this after a successful goto run.

The cleanest setup is:

1. finish the Phase 3 goto
2. leave the drone where it landed
3. start the circle demo from that new position

The circle is derived from the pillar keep-out, not from a free-space hardcoded center. With the current studio geometry it comes out to approximately:

- center `(-5.6181, 4.4087)`
- radius `4.0055 m`
- pillar clearance `1.2 m`
- altitude `2.0 m`

Before you call the start service, check the launch console for a line like:

- `Circle plan: source=keep-out 'studio_pillar' center=(-5.618, 4.409) radius=4.006 m`

If the printed center is near `(-0.136, 0.188)` or the radius is near `3.0 m`, stop and do not fly that run.

For the first real flight, use a slower circle speed than the default:

- `circle_speed_mps:=0.5`

Terminal 1:

```bash
ros2 launch drone_bringup position_circle_demo.launch.py \
  circle_altitude_m:=2.0 \
  circle_speed_mps:=0.5 \
  circle_use_keep_out_orbit:=true \
  circle_keep_out_name:=studio_pillar \
  circle_keep_out_clearance_m:=1.2 \
  max_circle_entry_distance_from_start_m:=8.0 \
  use_speed_profile:=false \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/cdrone3/demo/position_goto_abort std_srvs/srv/Trigger "{}"
```

Note:

- the circle launch reuses the goto demo node under the hood
- that is why the start and abort services are still `/demo/position_goto_start` and `/demo/position_goto_abort`

Expected result:

- take off
- switch to OFFBOARD
- move to a safe circle entry point
- fly one perimeter-safe loop
- hold briefly
- land

## Optional Landing Note

These demos hand landing off to PX4 `AUTO.LAND`.

If landing feels too aggressive, the main PX4 knob to lower first is:

- `MPC_LAND_SPEED`

This runbook does not change that parameter automatically.

## Simple Stop Rule

Stop the test sequence if any one of these happens:

- hover is unstable
- OFFBOARD engagement is inconsistent
- pose updates look stale or jumpy
- the goto demo aborts for perimeter or mode mismatch
- the landing behavior looks harsher than you want to accept

The right progression is:

1. hover
2. short goto
3. longer goto
4. circle

## Retired Goal

Do not reuse the old Phase 3 target:

- `goal_x_m:=4.1645`
- `goal_y_m:=0.1813`

That point lies on the east perimeter edge, not safely inside it.
