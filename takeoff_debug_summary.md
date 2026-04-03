# Takeoff Debug Summary

## Branch

- Active branch: `unstable_l2_realsense`

## Current Goal

- Get the scripted indoor hover demo working safely with OptiTrack external pose and PX4 position aiding.
- Immediate blocker: the demo is not consistently reaching a clean armed takeoff attempt because MAVROS/FCU state wobbles during startup and parameter sync.

## What Already Works

- OptiTrack -> `vrpn_mocap` -> repo external-pose adapter -> `/mavros/vision_pose/pose` is implemented and bench-validated.
- MAVROS is installed and sourced from `~/.bashrc`.
- PX4 external-vision params were set for bench testing:
  - `EKF2_GPS_CTRL = 0`
  - `EKF2_EV_CTRL = 3`
  - `EKF2_HGT_REF = 3`
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

## Main Outstanding Bug

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
