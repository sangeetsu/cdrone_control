# Current Problem: OFFBOARD Mode Blocked Indoors

## Status
- ✅ MAVROS connects to FC via mavlink-router UDP
- ✅ Arming works in STABILIZED mode (joystick throttle at zero)
- ✅ `keyboard_teleop_node` now defaults to MAVROS `ManualControl` for `ALTCTL` / `STABILIZED` indoor teleop
- ✅ Keyboard key `2` successfully switches PX4 into `ALTCTL`
- ✅ Keyboard teleop publishes centered `ManualControl` at idle (`z=500`)
- ✅ Keyboard arm/disarm commands now force manual throttle to `z=0` for 1.5s so arming is attempted with throttle low
- ✅ `mavros_velocity_node` streams setpoints at 20Hz
- ✅ Mode switch to OFFBOARD returns `mode_sent=True`
- ✅ Reproduced live on the drone computer: PX4 acknowledges the OFFBOARD request but stays in `AUTO.LOITER`
- ✅ `/mavros/local_position/pose` and `/mavros/local_position/velocity_body` are publishing, so MAVROS and EKF are not completely dead
- ✅ Current PX4 params confirmed over MAVROS: `EKF2_GPS_CTRL=7`, `COM_RC_IN_MODE=1`
- ✅ MAVROS `vision_pose` plugin is enabled already
- ⚠️ MAVROS `odometry` plugin is currently denylisted, so a proper `/mavros/odometry/out` VIO feed is not wired yet
- ❌ OFFBOARD mode does not activate — PX4 silently reverts or reports "resolve system health failures"
- ✅ Bench-only external-vision publisher added: `bench_vision_pose_node`
- ✅ One-command bench backend added: `ros2 launch drone_bringup bench_offboard.launch.py`
- ✅ Bench backend now reaches `mode: OFFBOARD` with the dummy vision pose active
- ✅ Sensors recalibrated and GPS module removed before the latest bench test
- ❌ Normal arming is still rejected after recalibration (`CommandBool success=False, result=1`)
- ❌ Bench force-arm still does not take; state remains `armed: false` even in `OFFBOARD`
- ❌ No real external-vision / VIO estimator exists in this repo yet

## Root Cause
**This is not GPS-specific; it is aiding-source specific.** PX4 can run without GPS, but velocity OFFBOARD still needs a supported state estimate source. Disabling GPS fusion alone does not create a replacement estimate for PX4 to trust. For no-GPS OFFBOARD, the practical paths are:

- external vision / VIO
- optical flow + rangefinder
- attitude/body-rate control instead of velocity OFFBOARD

## What Was Tried
- `CBRK_VELPOSERR` — does not exist in PX4 1.16.0 (removed in 1.14+)
- `COM_RC_IN_MODE=4` — set successfully, solved a separate "no manual control input" arming issue
- GPS module removed for the latest bench test to eliminate GPS-related confusion on the FC
- Force-arm (key `0`, MAV_CMD_COMPONENT_ARM_DISARM param2=21196) — retested during bench launch and still denied by PX4 system health failures

## System Info
- Hardware: Jetson Orin Nano + ARK PAB Carrier + ARKV6X + PX4 1.16.0
- Location: Indoor studio, no windows, no GPS signal
- MAVROS fcu_url: `udp://:14540@127.0.0.1:14550`
- mavlink-router: `/home/jetson/.local/bin/start_mavlink_router.sh` (NOT systemd)

## Options to Resolve (Not Yet Tried)

### Option A — Take outside for GPS fix (easiest)
Get a 3D GPS fix outdoors first, then test OFFBOARD. Not suitable for indoor bench testing.

### Option B — Disable GPS fusion only
In QGC → Parameters:
- Set `EKF2_GPS_CTRL` = `0`
- Reboot FC

This removes GPS fusion, but by itself it is not enough for no-GPS velocity OFFBOARD. PX4 still needs another aiding source.

### Option C — Bench-only dummy external vision pose
Add a bench-only ROS2 node that publishes a fixed pose to `/mavros/vision_pose/pose` so PX4 can treat MAVROS as an external vision source during props-off testing.

Notes:
- bench use only; the estimate does not move with the vehicle
- likely requires PX4 external-vision fusion params in QGC before OFFBOARD will engage
- good enough to validate the control path indoors without GPS

### Option D — Real stereo-derived VIO
Use the in-progress stereo vision stack as the basis for a real VIO / external-vision feed into PX4.

TODO:
- convert stereo stack output into a PX4-compatible external-vision stream
- decide whether to publish via MAVROS `vision_pose` or enable the MAVROS `odometry` plugin for `/mavros/odometry/out`
- tune PX4 external-vision fusion params once the real estimator is online

### Option E — Switch to attitude setpoints (code change)
Change `mavros_velocity_node.py` to publish to `/mavros/setpoint_raw/attitude` instead of `/mavros/setpoint_velocity/cmd_vel`. Attitude control bypasses the position controller entirely. User rejected this option.

## Recommended Next Session Plan
1. Use `ALTCTL` as the temporary indoor teleop mode from the Jetson keyboard path
2. Start MAVROS and keyboard teleop:
   `ros2 launch drone_bringup drone.launch.py`
   `ros2 run drone_control_pkg keyboard_teleop_node`
3. Press `2` for `ALTCTL`, then `1` to arm with throttle forced low
4. Confirm `/mavros/manual_control/send` is active if arming still fails:
   `ros2 topic echo /mavros/manual_control/send --once`
5. Long-term: replace the dummy node with stereo-derived VIO from the vision stack and return to proper OFFBOARD autonomy

## Resume Commands
```bash
# Temporary indoor teleop path
ros2 launch drone_bringup drone.launch.py

# In another terminal
ros2 run drone_control_pkg keyboard_teleop_node

# Check manual-control messages flowing:
ros2 topic echo /mavros/manual_control/send --once

# Keyboard sequence:
# 1. Press 2 -> ALTCTL
# 2. Press 1 -> arm (teleop forces throttle to zero first)
# 3. Use W/A/S/D, Q/E, R/F to fly indoors in ALTCTL

# OFFBOARD bench path remains available separately:
ros2 launch drone_bringup bench_offboard.launch.py
```
