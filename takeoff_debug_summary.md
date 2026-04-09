# Takeoff Debug Summary

## Branch

- Active branch: `unstable_l2_realsense`

## Current Goal

- Get the scripted indoor hover demo working safely with OptiTrack external pose and PX4 position aiding.
- Immediate blocker: the demo is not consistently reaching a clean armed takeoff attempt because MAVROS/FCU state wobbles during startup and parameter sync.

## What Already Works

- OptiTrack -> `vrpn_mocap` -> repo external-pose adapter -> `/mavros/vision_pose/pose` is implemented and bench-validated.
- MAVROS is installed and sourced from `~/.bashrc`.
- PX4 external-vision params were set for indoor flight:
  - `EKF2_GPS_CTRL = 0`
  - `EKF2_EV_CTRL = 11` (pos XY + Z + yaw on the `PoseStamped` pipeline)
  - `EKF2_HGT_REF = 3` (EV height)
  - `EKF2_MAG_TYPE = 5` (no magnetometer, use EV yaw)
- `keyboard_teleop_node` supports `POSCTL` on key `6`.
- `guided_target` MAVROS plugin was disabled to avoid irrelevant `PositionTargetGlobal ... no origin` warnings indoors.

## New Demo Path

- Main node: [ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py)
- Launch file: [ros2/src/drone_bringup/launch/position_hover_demo.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/position_hover_demo.launch.py)
- Runbook: [position_hover_demo_runbook.md](/home/jetson/cdrone_control/position_hover_demo_runbook.md)

## Current Launch Command

```bash
source ~/.bashrc
ros2 launch drone_bringup position_hover_demo.launch.py \
  pose_source:=optitrack \
  optitrack_server:=192.168.0.217 \
  rigid_body_name:=RigidBody3 \
  takeoff_altitude_m:=0.6 \
  takeoff_rate_m_s:=0.3 \
  takeoff_strategy:=AUTO_MODE \
  hover_duration_s:=5.0 \
  hover_mode:=HOLD \
  max_horizontal_excursion_m:=0.5
```

Start command:

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

Abort command:

```bash
source ~/.bashrc
ros2 service call /cdrone/drone01/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

## Important Code Changes Already Made

- External-pose contract is now generic instead of VIO-specific:
  - canonical input topic: `/cdrone/<drone_id>/external_pose/input_pose`
  - legacy alias remains: `/cdrone/<drone_id>/vio/input_pose`
- OptiTrack support:
  - [ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py)
  - [ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py)
  - [ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py)
- Hover demo:
  - ROS 2 param mirror path via `/mavros/param/get_parameters`, `/mavros/param/set_parameters`, `/mavros/param/pull`
  - relative-altitude checks instead of assuming local `z ~= 0`
  - `takeoff_strategy` parameter added
  - supported values:
    - `AUTO_MODE`
    - `LOCAL_COMMAND`
- Current default strategy for debugging is `AUTO_MODE`

## Takeoff Debug Timeline

### 1. Original scripted takeoff

- The demo originally used:
  - set mode `AUTO.TAKEOFF`
  - arm
  - wait for climb
- On props-on test, the vehicle armed but never audibly ramped past arm RPM.
- That made it unclear whether PX4 was actually engaging takeoff logic.

### 2. Explicit local takeoff experiment

- I added `/mavros/cmd/takeoff_local` support for a more explicit takeoff trigger.
- Result from live capture:
  - PX4 accepted arming
  - PX4 was in `AUTO.TAKEOFF`
  - `/mavros/cmd/takeoff_local` was rejected with `result 3`
- Relevant log:
  - [python3_10327_1775175576224.log](/home/jetson/.ros/log/python3_10327_1775175576224.log)
- Interpretation:
  - this firmware/stack appears to reject `MAV_CMD_NAV_TAKEOFF_LOCAL`
  - this is not a “not enough throttle from our node” problem

### 3. Switched back to `AUTO_MODE`

- I added `takeoff_strategy` and switched the active launch back to `AUTO_MODE`.
- I also added periodic takeoff-progress logging in the node so future captures show:
  - current altitude delta
  - target altitude delta
  - active mode

### 4. Latest retries

- The latest retries did not reach a clean takeoff attempt.
- They aborted during `SYNC_TAKEOFF_PARAM`, before arm/takeoff.
- Culprit symptoms:
  - brief `/mavros/state` flip to `connected: false`
  - MAVROS param pull wobble / timeout during startup

Relevant log:

- [python3_11245_1775175878147.log](/home/jetson/.ros/log/python3_11245_1775175878147.log)

Key lines:

- [python3_11245_1775175878147.log](/home/jetson/.ros/log/python3_11245_1775175878147.log#L2): start requested
- [python3_11245_1775175878147.log](/home/jetson/.ros/log/python3_11245_1775175878147.log#L3): requested FCU param pull
- [python3_11245_1775175878147.log](/home/jetson/.ros/log/python3_11245_1775175878147.log#L4): aborted due to `FCU disconnected`
- [python3_11245_1775175878147.log](/home/jetson/.ros/log/python3_11245_1775175878147.log#L6): param pull failed warning

Observed from `/mavros/state` capture:

- `connected: true` -> brief `connected: false` -> `connected: true` again
- The current demo safety logic treats that transient false sample as fatal.

### 5. Live capture on 2026-04-03

- A fresh live capture was recorded at:
  - [temp_outputs/live_takeoff_capture_20260403T004206Z](/home/jetson/cdrone_control/temp_outputs/live_takeoff_capture_20260403T004206Z)
- The newer demo instance accepted the start request and entered `SYNC_TAKEOFF_PARAM`.
- It requested the FCU param pull successfully, then aborted almost immediately with:
  - `lost local pose updates`
- Relevant node log:
  - [python3_14174_1775176868487.log](/home/jetson/.ros/log/python3_14174_1775176868487.log)
- Important bag evidence:
  - `/mavros/vision_pose/pose` stayed healthy at high rate through the trigger window.
  - `/mavros/local_position/pose` did not die completely, but it had a gap of about `0.53 s` right after the start request.
  - That was enough to trip the demo's `local_pose_timeout_s=0.5`.
- Extra complication from the machine state during capture:
  - two full `position_hover_demo.launch.py` stacks were running at once
  - old stack from about `00:24 UTC`
  - newer stack from about `00:41 UTC`
  - this made `/cdrone/drone01/demo/position_hover_state` oscillate between `IDLE` and `ABORT` and makes the capture environment untrustworthy until only one stack is left running

### 6. Live capture on 2026-04-03 after startup hardening

- A newer capture was recorded at:
  - [temp_outputs/position_hover_capture_20260403T005801Z](/home/jetson/cdrone_control/temp_outputs/position_hover_capture_20260403T005801Z)
- This run got materially farther than the earlier startup-failure captures:
  - `IDLE -> SYNC_TAKEOFF_PARAM -> SET_TAKEOFF_MODE -> ARMING -> TAKEOFF -> ABORT`
- Relevant node log:
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log)
- Key lines:
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L2): start requested
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L4): FCU param pull complete
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L8): `AUTO.TAKEOFF` confirmed
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L9): arm requested
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L11): vehicle armed and entered `TAKEOFF`
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L12): takeoff progress still near zero climb (`delta_z=-0.05 m`)
  - [python3_27058_1775177900573.log](/home/jetson/.ros/log/python3_27058_1775177900573.log#L13): abort because PX4 left takeoff mode and switched to `AUTO.LAND`
- Interpretation:
  - startup fragility is no longer the main blocker for this path
  - the current blocker is now post-arm PX4 behavior during the `AUTO.TAKEOFF` climb attempt
  - PX4 is internally dropping out of takeoff into land before any meaningful climb is observed

### 7. Root cause decoded from the same capture

- I decoded the PX4 `status_event` IDs from the same capture window.
- The critical post-arm sequence is:
  - `1775177946.791`: `Armed by external command`
  - `1775177946.832`: `Using default takeoff altitude: 0.60 m`
  - `1775177947.854`: `Manual control lost`
  - `1775177947.863`: `Failsafe activated: switching to Land`
- Relevant captured PX4/MAVROS state from the bag:
  - `COM_RC_IN_MODE = 1`
  - `COM_RC_LOSS_T = 0.5`
  - `NAV_RCL_ACT = 2`
  - `COM_RCL_EXCEPT = 0`
  - arming-check events also reported `Home position not set`
- Interpretation:
  - PX4 is not simply "refusing to ramp further" for no reason
  - it starts the takeoff spool/ramp, then loses its manual-control source, triggers the manual-control-loss failsafe, and exits to `AUTO.LAND`
  - because the capture also reports `Home position not set`, the configured RC-loss action is not practically recoverable as an RTL path in this indoor session, so the observed fallback is land
  - this makes the current blocker a PX4 manual-control / failsafe configuration issue, not a demo-node takeoff-command issue

### 8. Live capture on 2026-04-03 after setting `COM_RC_IN_MODE` to no stick input

- A newer capture was recorded at:
  - [temp_outputs/position_hover_capture_20260403T013355Z](/home/jetson/cdrone_control/temp_outputs/position_hover_capture_20260403T013355Z)
- This changed the failure mode in an important way:
  - the earlier `Manual control lost` -> `Failsafe activated: switching to Land` sequence disappeared
  - PX4 stayed in `AUTO.TAKEOFF`
  - the vehicle still never climbed
  - PX4 then auto-disarmed while still on the ground
- Relevant node log:
  - [python3_49627_1775180046233.log](/home/jetson/.ros/log/python3_49627_1775180046233.log)
- Relevant captured facts:
  - [python3_49627_1775180046233.log](/home/jetson/.ros/log/python3_49627_1775180046233.log#L47): `State ARMING -> TAKEOFF`
  - [python3_49627_1775180046233.log](/home/jetson/.ros/log/python3_49627_1775180046233.log#L48): first takeoff progress sample shows effectively zero climb
  - [summary.txt](/home/jetson/cdrone_control/temp_outputs/position_hover_capture_20260403T013355Z/summary.txt): final abort came from demo stage timeout because the node did not yet explicitly check for unexpected disarm during `TAKEOFF`
  - bag decode showed `/mavros/state.armed` changed from `True` to `False` about `11 s` after arming while mode stayed `AUTO.TAKEOFF`
  - bag decode showed `/mavros/extended_state.landed_state` stayed `1` the entire time
  - PX4 event `16017271` decoded to `Disarmed by preflight inaction`
  - captured params still included `COM_DISARM_PRFLT = 10.0`
- Interpretation:
  - changing `COM_RC_IN_MODE` removed the manual-control-loss abort path
  - the vehicle still never achieved takeoff or even a meaningful climb attempt
  - because PX4 still considered the vehicle landed, it auto-disarmed after the preflight-inactivity timeout
  - this means the current primary blocker is now the lack of actual climb / takeoff detection inside PX4, not RC-loss failsafe behavior

### 9. Frame alignment fix (2026-04-06) — RESOLVED

**Root cause of March 31 drift and crash (log_43, log_44):**

Two compounding issues were identified from ULG flight log analysis and live mocap axis testing:

1. **Heading instability (log_43, EKF2_EV_CTRL=3)**: PX4 used magnetometer for heading indoors, which wandered from -106° to -90° to -114°. This rotated the NED frame while the trajectory setpoint remained in the old frame, creating an ever-growing position error that PX4 tried to "correct" by flying sideways.

2. **180° rigid body heading error**: Live testing with Motive Z-up configuration showed the rigid body's defined "front" in Motive points at the drone's TAIL. The VRPN quaternion reported yaw ≈ -0.7° while the physical drone nose pointed in the Motive -X direction. With EV yaw fusion (EKF2_EV_CTRL=15), PX4 would think the nose faces the opposite direction, reversing every position correction.

3. **Wrong Motive axis change (log_44)**: User's Motive axis change for log_44 caused Z-axis inversion (EV height flipped from +1.126 to -0.795) and a 90° yaw jump during takeoff, leading to the crash.

**Fix applied:**

- **Software fix (portable to all drone clones)**:
  - `rpy_offset_rad` default changed from `[0.0, 0.0, 0.0]` to `[0.0, 0.0, 3.141593]` in `external_pose_px4_bridge.launch.py` (line 152)
  - The adapter node applies a 180° yaw rotation to the VRPN quaternion before forwarding to MAVROS
  - Motive configured to **Z-up** (right-handed), streaming via VRPN

- **PX4 parameters set**:
  - `EKF2_EV_CTRL = 15` (fuse EV position XY + Z + yaw)
  - `EKF2_MAG_TYPE = 5` (disable magnetometer, use EV yaw only)
  - `EKF2_HGT_REF = 3` (EV as height reference)
  - `EKF2_GPS_CTRL = 0` (GPS disabled)

- **QoS fix**: adapter node subscription changed to match vrpn_mocap's `BEST_EFFORT` reliability policy

**Verification (2026-04-06):**

Axis test with 180° yaw offset applied (baseline at rest: X=4.731, Y=0.541, Z=0.098, Yaw=177.1°):

| Test | Expected | Measured delta | Result |
|------|----------|----------------|--------|
| UP (~1ft) | dZ > 0 | dZ=+0.688, dX=+0.057, dY=+0.022 | PASS |
| RIGHT (~1ft) | dY > 0 | dY=+0.608, dX=+0.021, dZ=-0.007 | PASS |
| FORWARD (~1ft) | dX ≠ 0 | dX=-0.658, dY=+0.011, dZ=+0.002 | PASS |

All axes clean with near-zero crosstalk. Yaw stable at ~177° throughout. Frame alignment is correct.

### 10. Live Motive realignment on 2026-04-07 — yaw fixed, height still confusing

- Section 9's 180 degree software yaw offset was a temporary workaround.
- After the Motive ground plane was redefined and the rigid body was recreated with its local forward axis aligned to the drone nose, the software yaw workaround was removed again:
  - [ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py)
  - [ros2/src/drone_bringup/launch/position_hover_demo.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/position_hover_demo.launch.py)
- The standalone bridge launch default was also corrected to match the current Motive rigid body topic:
  - `rigid_body_name` now defaults to `RigidBody3`

**Current live PX4 params re-checked after FCU reboot:**

- `EKF2_EV_CTRL = 11`
- `EKF2_MAG_TYPE = 5`
- `EKF2_HGT_REF = 3`
- `EKF2_GPS_CTRL = 0`
- `EKF2_BARO_CTRL = 0`
- `EKF2_RNG_CTRL = 0`

**Live verification after the rigid body remake:**

- Raw `/vrpn_mocap/RigidBody3/pose` at the "nose along Motive +X" reference pose now reports yaw near `0 deg`.
- End-to-end pose chain is now aligned:
  - `VRPN -> /cdrone/drone01/external_pose/input_pose`: effectively exact match
  - `VRPN -> /mavros/vision_pose/pose`: effectively exact match
  - `VRPN -> /mavros/local_position/pose`: yaw agrees within about `1 deg`, XY within a few mm
- Ground-only arm/disarm test passed without sending a takeoff command:
  - arm accepted
  - disarm accepted

**Interpretation:**

- The old 180 degree heading/body-frame inconsistency is now resolved.
- The remaining PX4 estimator complaints are no longer best explained by a yaw-frame mismatch.

**Remaining live problems observed on 2026-04-07:**

- The OptiTrack/VRPN stream is still not perfectly continuous:
  - measured over `8 s`: average gap about `23.7 ms`
  - max gap about `276 ms`
  - `2` gaps exceeded `250 ms`
  - `17` gaps exceeded `100 ms`
- The adapter node reports matching source timeout warnings on `/vrpn_mocap/RigidBody3/pose`.
- In the full idle hover-demo stack, PX4 still intermittently reports:
  - `Navigation error: No valid position estimate`
  - `Navigation error: No valid global position estimate`
- Home/global-origin setup now succeeds, but the height readback is still confusing:
  - raw mocap / vision Z near the floor was about `+0.12 m`
  - `/mavros/local_position/pose.z` was about `+17.28 m`
  - `/mavros/home_position/home.position.z` was also about `+17.28 m`

**Current best interpretation of the height issue:**

- The large `~17.28 m` Z offset is probably not the same kind of estimator-fusion failure as the old yaw problem.
- It is more likely a reference / origin interpretation problem around:
  - the manually injected global origin + home position
  - MAVROS geographic altitude conversion
  - the difference between PX4 local origin, home position, and the Motive world frame
- In other words:
  - yaw/body alignment now looks correct
  - the remaining work is to make the height reference unambiguous and to reduce pose-stream dropouts

## Main Outstanding Bug

- The earlier drift/crash blocker (March 31 flights) has been **resolved** — see "9. Frame alignment fix" below.
- The repo startup path was a real issue, but it is no longer the immediate blocker in the latest captures.
- The earlier RC-loss blocker has also now been removed from the active failure path by changing `COM_RC_IN_MODE`.
- The current blocker is:
  - PX4 arms
  - PX4 remains in `AUTO.TAKEOFF`
  - the vehicle does not climb and remains in landed state
  - PX4 auto-disarms by `preflight inaction`
- The machine still also needs a single clean launch instance before any retest.

## Recommended Next Fixes

### Highest priority

- Repo-side startup hardening is now in place:
  - brief `FCU disconnected` blips are debounced during `SYNC_TAKEOFF_PARAM`
  - MAVROS param-pull wobble now retries instead of aborting immediately
  - short local-pose gaps during `SYNC_TAKEOFF_PARAM` get a longer timeout window
- Before the next retest, stop the stale older `position_hover_demo.launch.py` stack
  - only one demo node / bridge chain should be running
- Use the repo helper:
  - [scripts/capture_position_hover_debug.sh](/home/jetson/cdrone_control/scripts/capture_position_hover_debug.sh)
  - this is intended for observer-side capture while the operator launches/triggers separately

### After that

- The RC-loss issue is no longer the first thing to fix.
- The next PX4-side investigation should focus on why no actual takeoff begins:
  - hover-thrust / takeoff thrust tuning
  - takeoff state machine conditions
  - land detector behavior
  - any PX4-side requirement that still prevents a climb setpoint from being acted on
- Immediate signals to keep watching:
  - `/mavros/state.armed`
  - `/mavros/extended_state`
  - PX4 `status_event`
  - `COM_DISARM_PRFLT`
- If the same behavior repeats, the clearest concise diagnosis is:
  - `AUTO.TAKEOFF` accepted
  - no climb
  - still landed
  - disarmed by preflight inaction

## Notes About Safety

- The current demo intends to abort to `AUTO.LAND`, not to motor cut.
- That is repo behavior, not a guarantee against every PX4-side failure.
- The mocap stream is much better than the earlier bad session, but not perfectly gap-free.
- The netted hover that drifted on March 31 was archived here:
  - [flight_logs/2026-03-31T23-14-47-0700_phx_posctl_drift/explanation.md](/home/jetson/cdrone_control/flight_logs/2026-03-31T23-14-47-0700_phx_posctl_drift/explanation.md)

## Useful Files

- [ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/position_hover_demo_sequence_node.py)
- [ros2/src/drone_bringup/launch/position_hover_demo.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/position_hover_demo.launch.py)
- [ros2/src/drone_bringup/config/apm_pluginlists.yaml](/home/jetson/cdrone_control/ros2/src/drone_bringup/config/apm_pluginlists.yaml)
- [ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/external_pose_adapter_node.py)
- [ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py](/home/jetson/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/external_pose_bridge_node.py)

### 11. Pipeline validation on 2026-04-08 — known-good OptiTrack endpoint committed to repo config

- Added a dedicated bringup config file for the current indoor mocap setup:
  - [ros2/src/drone_bringup/config/optitrack_defaults.yaml](/home/jetson/cdrone_control/ros2/src/drone_bringup/config/optitrack_defaults.yaml)
- Both launch files now read their default OptiTrack and indoor-reference values from that config file:
  - [ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/external_pose_px4_bridge.launch.py)
  - [ros2/src/drone_bringup/launch/position_hover_demo.launch.py](/home/jetson/cdrone_control/ros2/src/drone_bringup/launch/position_hover_demo.launch.py)
- The known-good defaults currently stored there are:
  - `optitrack_server = 192.168.0.217`
  - `rigid_body_name = RigidBody3`
  - `global_origin_altitude_m = 17.1637`
  - `vrpn_sensor_data_qos = true`
  - `source_best_effort = true`

**Live validation results:**

- `localhost:3883` is not the active OptiTrack server on this machine.
  - direct check to `localhost:3883` failed
  - direct check to `192.168.0.217:3883` succeeded
- With `optitrack_server:=192.168.0.217`, the VRPN client created `RigidBody3` immediately and the bridge path became live again.

**End-to-end pose alignment with the real OptiTrack server:**

- Debug summary from `/cdrone/drone01/external_pose/debug/summary` showed:
  - `source -> adapter`: exact within report precision
  - `source -> vision`: exact within report precision
  - `source -> local`: about `dx=+0.0007 m`, `dy=-0.0008 m`, `dz=+0.0119 m`, `dyaw=+0.04 deg`
- Derived indoor origin math is now self-consistent:
  - `global_origin.altitude_m = 17.1630`
  - `derived_origin_altitude_m = 17.1629`
  - `origin_altitude_error_m = 0.0001`

**Transport / QoS check:**

- Best-effort performed better than reliable on this network with the current OptiTrack stream.
- Best-effort sample over `8 s`:
  - `348` samples
  - average gap `0.0231 s`
  - max gap `0.1613 s`
  - `6` gaps over `0.10 s`
  - `0` gaps over `0.25 s`
- Reliable sample over `8 s`:
  - `344` samples
  - average gap `0.0224 s`
  - max gap `0.4962 s`
  - `21` gaps over `0.10 s`
  - `4` gaps over `0.25 s`
- Interpretation:
  - keep the OptiTrack path on `vrpn_sensor_data_qos = true`
  - keep the adapter source subscription on `source_best_effort = true`
  - the frame and origin alignment now look clean
  - the remaining transport risk is occasional real VRPN stalls from the upstream stream, not a bridge-side frame bug

**Ground-only arm check after the config / pipeline fixes:**

- Bench validation was run from the standalone external-pose bridge path only.
- No hover-demo start request or takeoff command was sent.
- Observed MAVROS state before arming:
  - `connected = true`
  - `armed = false`
  - `mode = AUTO.LOITER`
  - `manual_input = false`
- MAVROS arm service result:
  - `success = true`
  - `result = 0`
- MAVROS disarm service result:
  - `success = true`
  - `result = 0`
- FCU status text during the arm/disarm window:
  - `Armed by external command`
  - PX4 log file announcement
  - `Disarmed by external command`
- No new estimator or prearm rejection messages appeared during that no-flight arm test.

### 12. Demo milestone on 2026-04-08 — first successful autonomous arm and partial takeoff

- The hover demo finally armed and lifted off under the full demo sequence.
- Sequence observed in [output.txt](/home/jetson/cdrone_control/output.txt):
  - `IDLE -> SYNC_TAKEOFF_PARAM`
  - `SYNC_TAKEOFF_PARAM -> SET_TAKEOFF_MODE`
  - `SET_TAKEOFF_MODE -> ARMING`
  - arm request accepted
  - `ARMING -> TAKEOFF`
  - PX4 reported `Using default takeoff altitude: 0.60 m`
  - PX4 reported `Takeoff detected`
- The highest takeoff progress reported by the demo before it stopped was:
  - `delta_z = 0.22 m`
  - `target_delta_z = 0.60 m`

**Important interpretation from the log:**

- The demo did **not** stop because of the demo stage timeout.
  - The stage timeout default is `30 s`, and this attempt aborted only a few seconds into `TAKEOFF`.
- The log does **not** show any battery warning, low-battery event, or battery failsafe message.
- The immediate stop condition in this run was:
  - PX4 mode changed from `AUTO.TAKEOFF` to `AUTO.LOITER`
  - then the demo aborted with `lost takeoff mode during climb: AUTO.LOITER`
  - then the demo commanded `AUTO.LAND`

**What is proven vs not yet proven:**

- Proven:
  - the repo-side demo sequence can now arm and initiate takeoff
  - PX4 detected takeoff
  - the run was cut short by the mode change to `AUTO.LOITER`, not by the configured experiment timeout
- Not yet proven from this log alone:
  - why PX4 switched to `AUTO.LOITER` at that moment
  - whether the vehicle would have continued climbing or stabilizing if the demo had not immediately aborted to `AUTO.LAND`
  - whether low battery contributed indirectly, because there is no explicit battery event in this log

**Next debugging implication:**

- The primary remaining issue is no longer “cannot arm”.
- The next issue to resolve is the handoff around the top of `AUTO.TAKEOFF`:
  - either PX4 is leaving `AUTO.TAKEOFF` earlier than expected
  - or the demo is treating a legitimate PX4 `AUTO.TAKEOFF -> AUTO.LOITER` transition as a failure before it has enough altitude evidence to accept it
