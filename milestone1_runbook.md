# Milestone 1 Runbook

## Current hover route: ZED 2i without mocap

Use `localization_source:=zed` for the 1 m, 20 s hover milestone. PX4 1.16.0
is configured to fuse airframe-center pose from the ZED 2i; this route does not
use optical-flow or range handoff. `localization_source:=onboard` selects the older ARK Flow path
and its MAVROS configuration **disables vision pose**. Do not run both hover
launches at once. The launch default, `external_pose`, is the legacy mocap path.

**Current bench status:** the FCU parameters matched the ZED profile on October
8, 2026. A ZED run then aborted in `WARMUP` before arming because PX4's
predictive relative-position status bit briefly cleared. Bench monitoring
reproduced 0.6 s dips about every 31 s while current horizontal position,
velocity and ZED frames stayed valid. The controller now tolerates a dip under
one second only while those current estimates and camera data remain healthy;
sustained loss still aborts. The candidate mount file below was generated using
a ruler-measured translation; its report contains no accepted pivot fit. Manual
X/Y/Z/yaw and PX4 fusion checks are still required. These observations do
**not** clear the drone for flight.

**Latest no-arm abort (October 10, 2026 UTC):** the ZED hover launch began at
22:42:39 UTC. The controller reached `ARMING`, but the MAVROS arm service
returned `success=False, result=1` (temporarily rejected) at 22:43:44 UTC.
PX4 emitted "Arming denied: Resolve system health failures first." Its
arming-check events, decoded with the event metadata embedded in the previous
FCU log, reported no manual control input and no connection to the ground
control station for all modes, including OFFBOARD, before the arm request. PX4
did not report a valid
GCS heartbeat at that moment; the MAVROS companion connection alone did not
satisfy this check. Other checks for missing global position, home and mission
applied to mission/RTL/loiter, not OFFBOARD. The "No offboard signal" check
was absent from the arm-time reports; it appeared before the sequence and again
after the controller stopped setpoints. The controller briefly selected
`AUTO.LAND` as abort cleanup, but the FCU remained disarmed and never entered
`TAKEOFF`. Verify a stable QGroundControl or RC link and inspect QGC's
Arming and Preflight Checks before any later supervised attempt; do not disable
the link/arming failsafe to make the check pass. The launch was stopped after
confirming the FCU was disarmed. No flight was attempted during this diagnosis.

**Earlier no-liftoff attempt (October 9, 2026 UTC):** the launch at 00:26:58 UTC
became ready, requested OFFBOARD and armed PX4. The controller entered
`TAKEOFF` at 00:28:01 UTC, then requested `AUTO.LAND` at 00:28:06 because ZED
height was still less than 3 cm above its starting height after the five-second
no-liftoff limit. PX4 reported "Takeoff detected" at 00:28:01, which is its
land-detector state and does not prove a physical rise. PX4 later reported
landing and disarm. The ESCs were powered and propellers spun, but actual thrust
and supply voltage under load were not measured. Keep this abort as a stop
condition until the takeoff command and hardware are checked on the bench. An
aborted controller now requires a fresh launch before another sequence.

The recovered PX4 ULog is `/home/jetson/no_liftoff_2026-10-08.ulg`. It confirms
the motors were commanded and the ZED height observation stayed nearly flat.
The old takeoff code held its position target about 0.25 m above the measured
position. With PX4 configured for a one-second spool-up and three-second takeoff
ramp, the logged collective thrust setpoint peaked at about 0.58 before the
five-second watchdog fired. The stored `MPC_THR_HOVER` is 0.62, but
`MPC_USE_HTE=1` lets PX4 use its [hover-thrust estimator](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference#MPC_USE_HTE), whose estimate was not
recorded in this log. The small position-target lead and short watchdog may
have contributed to the no-lift attempt; the log cannot establish the sole
cause. The logged setpoint is not a measurement of propeller thrust. Before a
later flight attempt, inspect propeller fit and orientation, verify motor
direction with propellers removed using [PX4's actuator checks](https://docs.px4.io/v1.16/en/config/actuators), and check the supply voltage under load on
the bench. The ZED path
now allows at most 0.6 m of position-target lead and eight seconds for ZED to
confirm 3 cm of rise. The ARK Flow path retains its earlier limits. **The new
values have only been unit-tested; do not treat this as flight validation.**
The ZED launch arguments are `zed_takeoff_setpoint_lead_m` (default `0.6`)
and `zed_takeoff_no_lift_timeout_s` (default `8.0`);
`takeoff_no_lift_timeout_s` still applies to the ARK Flow path.

**Current Jetson reset blocker (October 9, 2026 UTC):** two ZED hover
launches coincided with abrupt Jetson resets. The ROS launches began at
00:10:30 and 00:15:50 UTC; the next boots began at 00:11:18 and 00:16:41 UTC.
The last boot reports `SYS_RESET_N` in
`/sys/devices/platform/bus@0/c360000.pmc/reset_reason`. The preceding journals
show no orderly shutdown, OOM kill or thermal shutdown. This reset indication
does not identify which supply, cable, carrier-board circuit or reset input
caused it; see [NVIDIA's reset-reason mapping](https://forums.developer.nvidia.com/t/how-to-get-the-power-up-reason-on-orin/241365).
A laptop SSH disconnect during these attempts is a consequence of
the Jetson restarting. QGroundControl/PX4 parameter changes will not fix a
Jetson reset. **Do not attempt the hover or run another camera launch with
flight hardware connected until the reset cause is resolved.**

With motors/ESCs disconnected, inspect the Jetson input supply and connector,
camera USB cable/port, and any shared battery regulator. Verify that the supply
can support the Jetson's 25 W mode plus connected devices without voltage drop.
After correcting the suspected power/reset path, build and source the workspace
as in section 1, then isolate the camera load first:

```bash
cat /sys/devices/platform/bus@0/c360000.pmc/reset_reason
journalctl --list-boots --no-pager | tail -5
ros2 run drone_control_pkg zed_vio_node --ros-args \
  -p mount_config:=/home/jetson/zed_hover_mount_run2.yaml
```

Keep the ZED-only node running on the bench for at least five minutes; verify
that `zed/tracking_status` remains `OK` and the Jetson does not restart. Then
stop it and run the flight-disabled diagnostics launch in section 2 for another
five minutes. If either step resets the Jetson, stop and inspect the latest
`reset_reason`, previous-boot journal, and hardware power path before retrying.
The flight command in section 3 remains blocked until both steps and all pose
and PX4 fusion checks pass.

### 1. Build and inspect the FCU

With the motors/ESCs disconnected, stop any running `position_hover_demo`
launch. Build the current code and source it in each new terminal:

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
cd /home/jetson/cdrone_control
```

Export a full QGroundControl parameter backup. With the FCU disarmed, compare
its current values to the [ZED PX4 1.16 profile](ros2/src/drone_bringup/config/zed_hover_px4_1_16.yaml):

```bash
python3 scripts/inspect_onboard_localization.py \
  --profile ros2/src/drone_bringup/config/zed_hover_px4_1_16.yaml \
  --port udpin:127.0.0.1:14569
```

Use the `companion_aux` endpoint above with one inspection client at a time. The
target settings are `EKF2_EV_CTRL=11` (vision horizontal position, height, yaw),
`EKF2_HGT_REF=3`, `EKF2_OF_CTRL=0`, and `EKF2_RNG_CTRL=0`; keep the other values
in the profile. `11` does not request direct vision velocity. A matching
parameter profile only permits fusion; PX4 still needs fresh ZED poses. Apply
any differences in QGroundControl while disarmed, reboot, then inspect again.

### 2. Start ZED diagnostics, with flight disabled

Use the existing mount candidate only for the motors-disconnected bench check.
Its report at `/home/jetson/zed_hover_mount_run2.report.json` records
`method: ruler_translation` and `pivot_fit: null`. The packaged mount template
is `calibrated: false`, so omitting `zed_mount_config` will prevent pose output.
If the camera mount has moved, measure it again before using this command.
Use the resulting, bench-validated mount file in both launch commands below.

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=zed enable_flight:=false \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  initial_climb_enabled:=false
```

In separate sourced terminals, inspect the camera, PX4 estimate and readiness:

```bash
ros2 topic echo --once /cdrone/cdrone3/zed/tracking_status
ros2 topic hz /cdrone/cdrone3/zed/pose_cov
ros2 topic echo --once /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/mavros/local_position/pose
ros2 service call /cdrone/cdrone3/demo/position_hover_check std_srvs/srv/Trigger "{}"
ros2 topic echo /cdrone/cdrone3/demo/position_hover_diagnostics
```

Stop each continuous `ros2 topic` command with Ctrl+C before using that
terminal for another command. Expect ZED tracking status `OK`, fresh pose
updates, valid horizontal position and velocity in `estimator_status`, and a
successful `position_hover_check` after its continuous readiness period. The
diagnostics should say `source=zed`; `source=ark_flow` means the wrong launch is
running. The numerical `vx`/`vy` fields in a local-position message are not
proof of validity when estimator horizontal-velocity status is false.

In QGroundControl's **MAVLink Console** (not a shell), confirm PX4 is actually
fusing vision:

```text
listener estimator_status_flags -n 1
```

Require `cs_ev_pos`, `cs_ev_hgt`, and `cs_ev_yaw` to remain true. The
`position_hover_check` verifies live pose and PX4 position/velocity validity;
this PX4 check identifies the aiding source directly. Do not start a flight on a
parameter match alone.

With motors still disconnected, mark the airframe-center start position. Move
the whole drone by known positive body X (forward), positive body Y (left), and
positive Z (up) distances, then rotate through a known yaw and return to the
start. Watch `/cdrone/cdrone3/zed/pose_cov` and
`/cdrone/cdrone3/mavros/local_position/pose` during each move. Compare their
changes after accounting for the starting yaw: directions and distances must
agree with physical motion, and both heights must return to their starting
values without jumps or tracking loss. Repeat the PX4 fusion
check during and after motion. If any direction, scale, yaw, return-to-start,
tracking, or fusion check fails, fix that issue on the bench and repeat.

### 3. Supervised 1 m hover, only after the bench gate passes

Reconnect the flight hardware and use the physical test area, operator
takeover procedure and QGroundControl monitoring. Keep the validated ZED mount
and PX4 profile. Stop the diagnostics launch, then start the flight-enabled
launch in a sourced terminal:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=zed enable_flight:=true \
  zed_mount_config:=/home/jetson/zed_hover_mount_run2.yaml \
  initial_climb_enabled:=false \
  takeoff_altitude_m:=1.0 takeoff_rate_m_s:=0.3 hover_duration_s:=20.0
```

Wait for ZED tracking `OK`, sustained PX4 vision fusion, valid local position
and a successful readiness response:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_check std_srvs/srv/Trigger "{}"
```

Only if that call reports `success: true`, start the hover. Monitor its state
in another sourced terminal:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_start std_srvs/srv/Trigger "{}"
ros2 topic echo /cdrone/cdrone3/demo/position_hover_state
```

The start command can arm and take off. Keep the operator ready to take over.
To request an abort and `AUTO.LAND`:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

Accept this milestone only after takeoff, the full 20 s hover, landing and
disarm complete without estimator loss, pose jumps or visible unstable motion.
Do not use the ARK Flow initial-climb commands below with the ZED profile.

---

## Legacy ARK Flow investigation (not the current hover route)

This earlier onboard test uses **ARK Flow over CAN + PX4's IMU,
barometer and calibrated heading source**. PX4 performs sensor fusion; the
Jetson sends local position setpoints through MAVROS, with an optional initial
attitude-and-thrust climb when flow cannot operate at ground clearance. Its
parameter profile conflicts with the current ZED hover profile.

The hover launch has these paths:

| Argument | Behavior |
| --- | --- |
| `localization_source:=onboard` | MAVROS + local OFFBOARD hover; no mocap, pose injection, fixed global origin or home setup |
| `localization_source:=zed` | ZED airframe-center vision pose + MAVROS local OFFBOARD hover; current hover route above |
| `localization_source:=external_pose` | Original mocap/external-pose hover; compatibility default |
| `enable_flight:=false` | Onboard or ZED diagnostics only; no setpoints, arming or mode commands; default |
| `enable_flight:=true` | Onboard or ZED start service may command takeoff/hover/landing after readiness checks |
| `initial_climb_enabled:=true` | Onboard: climb using downward range and PX4 attitude control, then acquire position hold; default false |

`enable_flight` applies to the onboard and ZED paths. It does **not** make the legacy
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
It enables CAN flow/range subscriptions, uses the range sensor as the height
reference with continuous fusion over a level indoor floor, keeps barometer
fusion enabled, disables external vision/GNSS fusion, and sets
OFFBOARD-loss action to **Land** after one second. Sensor limits match the
standard ARK Flow, not ARK Flow MR. Calibration, mounting offsets and controller
gains are retained.

**Firmware prerequisite for this profile:** the current cdrone3 PX4 build
reports commit `6ea3539`. Its optical-flow startup condition cannot be met with
`EKF2_HGT_REF=2` (range height reference) and no other horizontal aid: range
height does not mark the terrain estimate valid, while horizontal aiding cannot
become active until optical flow starts. PX4 fixed this in the
[1.16 backport](https://github.com/PX4/PX4-Autopilot/pull/26961), included in
[v1.16.2](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.2). Use an
ARKV6X-compatible PX4 build containing that fix (the documented board target is
[`ark_fmu-v6x_default`](https://docs.px4.io/v1.16/en/flight_controller/ark_v6x))
before expecting this profile to pass `position_hover_check`. Back up FCU
parameters before updating firmware;
recheck calibration and all profile values afterward. Do not bypass the ROS
horizontal-aiding check or use the initial-climb path to work around this bug.

**If firmware remains unchanged:** use the separate
[barometer-reference bench profile](ros2/src/drone_bringup/config/onboard_optical_flow_baro_bench_profile.yaml)
to test optical-flow fusion only. It keeps `EKF2_HGT_REF=0` and
`EKF2_RNG_CTRL=1`, the combination recorded in LOG0343. That log showed
`cs_opt_flow=True` and recent optical-flow fusion on the current firmware, so
this is an observed way around the startup deadlock. The same log showed the
PX4 vertical estimate drifting several metres while downward range remained
near launch clearance, even though range-height fusion was reported active.
Therefore passing optical-flow fusion in this mode does not clear the vertical
estimator or qualify a hover. With ESCs disconnected, compare local Z, barometer,
and measured downward range during stationary holds and known manual lifts;
stop at the bench if they disagree. The separate ZED pose route is documented
at the top of this runbook and has its own bench gate.

The inspector defaults to the range-reference profile. For the no-firmware
bench route, select the barometer profile explicitly when reviewing current
values or printing commands:

```bash
python3 scripts/inspect_onboard_localization.py \
  --profile ros2/src/drone_bringup/config/onboard_optical_flow_baro_bench_profile.yaml \
  --print-commands
```

Do not apply the range-reference profile until the launch geometry is measured:
an earlier cdrone3 measurement found about 0.07 m sensor clearance, below ARK
Flow's 0.08 m working minimum, while the latest log reported roughly 0.17 m
initial range. Complete the physical clearance and bench checks in section 6
first. The profile is for a validated, level-floor setup.

Verify the actual downward sensor orientation and measured sensor offsets. The
inspection found `SENS_FLOW_ROT=0` and `EKF2_OF_POS_X=0.06 m`; retaining those
values does not verify their physical correctness. A calibrated heading source
is still required when external-vision yaw is disabled.

### 3. Operator Applies The Reviewed Configuration

Print the exact console commands **without connecting to the FCU**:

```bash
python3 scripts/inspect_onboard_localization.py --print-commands
```

Only after the section 6 clearance and bench checks pass, with the FCU
disarmed, apply the reviewed values in QGroundControl's parameter editor, or
enter the printed `param set ...` lines and `param save` in its MAVLink console.
Reboot the FCU using QGroundControl, then inspect again:

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
  surface texture/lighting, the reviewed FCU profile, and the firmware prerequisite
  above. In PX4 1.16,
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

**October 4 bench result:** the current FCU receives optical-flow observations
with `innovation_rejected=False`, but `time_last_fuse=0` and `fused=False`.
ROS range was about 0.16 m and flow quality 113, while the current FCU reported
`EKF2_HGT_REF=2`, `EKF2_OF_CTRL=1`, and `EKF2_GPS_CTRL=0` in the reviewed
configuration. This matches the unpatched firmware startup deadlock above.
`cs_yaw_align=False` also appeared, but yaw alignment is not a direct condition
in PX4's optical-flow startup check. Recheck heading health separately after
the fusion fix. The PX4 console accepts command lines such as the three
`listener ...` lines above; do not paste the printed `TOPIC:` output back at
the `nsh>` prompt.

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

The latest available flight ULog records `SENS_BOARD_ROT=27`. In PX4 1.16,
[rotations 10 and 27 apply the same vector mapping](https://github.com/PX4/PX4-Autopilot/blob/v1.16.0/src/lib/conversion/rotation.h#L1313-L1327),
and the logged level attitude was near zero roll/pitch. Do not change board
rotation solely because the stored number differs from the earlier inference;
verify the attitude against physical airframe motion.

Historical operator correction for a physically incorrect attitude, with ESCs
disconnected and the FCU disarmed (not indicated by the latest ULog):

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

**Current cdrone3 clearance gate:** an earlier operator measurement found about
0.07 m from the ARK Flow sensor face to the floor.
[ARK specifies](https://docs.arkelectron.com/products/sensor/ark-flow)
an optical-flow working distance starting at 0.08 m; the latest ULog recorded
`UAVCAN_RNG_MIN=0.05` and `SENS_FLOW_MINHGT=0.05`. LOG0343 reported roughly
0.17 m initial downward range, but the physical launch clearance for that run
was not independently measured here. A reported range alone does not verify
that the optical standoff meets the sensor specification. Do not use the flight
example below until the current geometry is measured, and do not treat the
raised initial-climb abort limits as a fix.

Without changing the airframe, a stable launch fixture may support the landing
feet while leaving an unobstructed, textured floor beneath the sensor. Measure
the distance from the sensor to the surface it actually sees; a solid pad under
the sensor leaves that distance unchanged. For margin, aim for at least 0.12 m
of measured clearance. With ESCs disconnected, compare raw range with measured
heights at several positions, confirm PX4 optical-flow fusion and valid local
position while stationary, and confirm local altitude returns to its starting
value after a gentle manual lift and return. Investigate the last flight's
accelerometer clipping and local-altitude/range disagreement before relying on
autonomous landing. Restore the manufacturer's 0.08 m flow/range minimums only
after verifying the physical clearance and current FCU parameters. If ground
position aiding then passes `position_hover_check` with
`initial_climb_enabled:=false`, that position-control path avoids the unvalidated
unaided thrust climb. This airframe carries a ZED 2i, not a D455. The ZED
vision-pose route above is now the selected hover path, subject to its separate
manual-motion and PX4 fusion checks. The retained D455 VSLAM bridge is not a
usable alternative on this airframe.

**Latest attempted flight (LOG0343, October 4):** position control was selected
from the start (`initial_climb_enabled:=false`), but the drone did not physically
lift. PX4 local altitude rose roughly 1.34 m by the AUTO.LAND request while
downward range stayed near launch clearance (about 0.17 m initially and 0.10 m
near abort). The raw barometer was nearly unchanged at abort, so the log does
not establish barometric pressure as the cause. The ROS node now requires
range-confirmed climb to enter HOVER, aborts when local altitude and downward
range disagree by over 0.35 m for 0.5 s, and requests landing if takeoff has
no 3 cm range-confirmed lift after 5 s. These checks detect the logged failure
mode; they do not repair the PX4 estimator or prove adequate thrust.

After confirming at least 0.12 m physical clearance, an unobstructed level
floor, and accurate live range readings at several measured heights with the
motors disconnected, use QGroundControl **Vehicle Setup → Parameters** while
disarmed and export a full backup. For this flat-floor test, apply these PX4
1.16 values and reboot if QGC requests it:

| Parameter | LOG0343 value | Flat-floor value | Purpose |
| --- | ---: | ---: | --- |
| `EKF2_HGT_REF` | 0 | 2 | Make range the reference for local height |
| `EKF2_RNG_CTRL` | 1 | 2 | Fuse range continuously while it remains valid |
| `EKF2_BARO_CTRL` | 1 | 1 | Retain barometer as another height source |
| `UAVCAN_RNG_MIN` | 0.05 m | 0.08 m | Restore the ARK Flow minimum |
| `SENS_FLOW_MINHGT` | 0.05 m | 0.08 m | Restore the ARK Flow working-height minimum |

Keep `EKF2_OF_CTRL=1` and `UAVCAN_SUB_FLOW=1`, `UAVCAN_SUB_RNG=1`; those were
already enabled. PX4 documents range as the indoor flat-terrain height
reference and warns that obstacles or uneven ground can make range fusion
inconsistent ([EKF2 guidance](https://docs.px4.io/v1.16/en/advanced_config/tuning_the_ecl_ekf),
[ARK Flow setup](https://docs.px4.io/v1.16/en/dronecan/ark_flow)). With motors
disconnected, check that local Z follows a known manual lift and returns to its
starting value; check optical-flow fusion and review IMU vibration/clipping
before another supervised flight. Do not raise the altitude overshoot or
vertical-speed abort limits to hide estimator disagreement.

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  localization_source:=onboard enable_flight:=true \
  takeoff_altitude_m:=1.0 takeoff_rate_m_s:=0.3 hover_duration_s:=20.0 \
  max_horizontal_excursion_m:=0.5 \
  initial_climb_enabled:=false
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

Expected progression with position control from the start: WARMUP (20 Hz for
1.5 s), SET_OFFBOARD, ARMING, TAKEOFF, HOVER, LANDING, COMPLETE. The hover
timer begins after local altitude and downward range are both within 0.15 m
of the requested height for one second, with PX4 reporting IN_AIR. X/Y and yaw
remain anchored to start.
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
| `initial_climb_max_height_m` | 1.25 m above the measured launch surface; hard abort limit, not a target |
| `initial_climb_rate_m_s` | 0.2 m/s commanded climb-speed cap |
| `initial_climb_max_thrust` | 0.75 normalized collective thrust ceiling |
| `initial_climb_thrust_slew` | Thrust increases by at most 0.25 per second |
| `initial_climb_min_range_m` | 0.15 m minimum tilt-corrected range for handover |
| `initial_climb_aiding_dwell_s` | 1 s continuous valid aiding and settled height |
| `initial_climb_no_lift_timeout_s` | Land if measured climb remains below 3 cm after 5 s |
| `initial_climb_timeout_s` | Land if handover is not ready within 8 s |
| `initial_climb_max_tilt_rad` | 0.2 rad (11.5 degrees) measured tilt limit |
| `initial_climb_max_speed_m_s` | 1.0 m/s measured vertical speed abort limit; the 0.2 m/s commanded climb rate remains unchanged |

The height and speed abort defaults were raised after the September 27 flight
logs showed an initial-climb height abort and, on the preceding attempt, a
vertical-speed abort. These limits allow more deviation during the unaided
phase; they do not change the controller or establish a successful hover.
Confirm that this test site has sufficient vertical and lateral clearance for
the full 1.25 m unaided-climb envelope and for motion while PX4 takes over
landing. The latest landing log also showed local-altitude drift despite a
near-ground range reading; inspect that separately before relying on autonomous
landing behavior.

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
`initial_climb_max_height_m`, vertical speed exceeding
`initial_climb_max_speed_m_s`, stale/invalid required data, unexpected yaw change,
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

Before a legacy mocap hover or goto test, restore its validated FCU baseline.
Do not run this restore script for the ZED route above; it changes PX4 fusion
parameters:

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
