# Current Problem: OFFBOARD Mode Blocked Indoors

## Status
- ✅ MAVROS connects to FC via mavlink-router UDP
- ✅ Arming works in STABILIZED mode (joystick throttle at zero)
- ✅ `mavros_velocity_node` streams setpoints at 20Hz
- ✅ Mode switch to OFFBOARD returns `mode_sent=True`
- ❌ OFFBOARD mode does not activate — PX4 silently reverts or reports "resolve system health failures"

## Root Cause
**No GPS fix indoors (studio has no windows).** PX4's EKF2 requires a valid local position estimate before allowing OFFBOARD mode. Without GPS, EKF2 stays in "constant position mode" and marks position estimate as invalid, blocking the position controller that OFFBOARD depends on.

## What Was Tried
- `CBRK_VELPOSERR` — does not exist in PX4 1.16.0 (removed in 1.14+)
- `COM_RC_IN_MODE=4` — set successfully, solved a separate "no manual control input" arming issue
- GPS module plugged in — module detected by PX4, but no fix possible indoors without windows
- Force-arm (key `0`, MAV_CMD_COMPONENT_ARM_DISARM param2=21196) — bypasses arming checks, but OFFBOARD switch still fails

## System Info
- Hardware: Jetson Orin Nano + ARK PAB Carrier + ARKV6X + PX4 1.16.0
- Location: Indoor studio, no windows, no GPS signal
- MAVROS fcu_url: `udp://:14540@127.0.0.1:14550`
- mavlink-router: `/home/jetson/.local/bin/start_mavlink_router.sh` (NOT systemd)

## Options to Resolve (Not Yet Tried)

### Option A — Take outside for GPS fix (easiest)
Get a 3D GPS fix outdoors first, then test OFFBOARD. Not suitable for indoor bench testing.

### Option B — Disable GPS fusion, use baro-only EKF
In QGC → Parameters:
- Set `EKF2_GPS_CTRL` = `0`
- Reboot FC

EKF2 initializes immediately with barometer + IMU. No GPS needed.
This is the recommended path for indoor OFFBOARD bench testing.

### Option C — Switch to attitude setpoints (code change)
Change `mavros_velocity_node.py` to publish to `/mavros/setpoint_raw/attitude` instead of `/mavros/setpoint_velocity/cmd_vel`. Attitude control bypasses the position controller entirely — works without any EKF position validity. W/S = pitch, A/D = roll, R/F = thrust. User rejected this option.

### Option D — Use local position from vision/mocap
Feed a fake or real position estimate via `/mavros/vision_pose/pose` to satisfy EKF2. Complex, not warranted for a bench test.

## Recommended Next Session Plan
1. Try **Option B** first: set `EKF2_GPS_CTRL=0` in QGC, reboot FC, retry OFFBOARD
2. Confirm with: `ros2 topic echo /mavros/state --once` — should show `guided: True` after arming + OFFBOARD switch
3. If that works, test WASD motor response

## Resume Commands
```bash
# Terminal 1
ros2 launch drone_bringup drone.launch.py

# Terminal 2
ros2 run drone_control_pkg mavros_velocity_node

# Terminal 3
ros2 run drone_control_pkg keyboard_teleop_node

# Check setpoints flowing:
ros2 topic hz /mavros/setpoint_velocity/cmd_vel   # should be ~20Hz

# Arm sequence:
# 1. Arm via joystick (throttle at zero) in STABILIZED
# 2. Press 3 in keyboard_teleop → OFFBOARD
# 3. Press W → motors should respond
```
