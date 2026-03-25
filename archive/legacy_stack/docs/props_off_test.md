# Props-Off Bench Testing Guide

**Current recommended indoor bench method:** use `keyboard_teleop_node` in `ALTCTL` via MAVROS `ManualControl`.

This replaces the older indoor teleop flow that used `mavros_velocity_node` and `OFFBOARD`. Keep the props off for all tests in this document.

## Safety First

- Remove all propellers before connecting power.
- Keep the airframe restrained on the bench.
- Keep one hand ready to disarm with key `4`.
- Use short command taps and press `SPACE` to recenter sticks after each test.
- Treat `0` force-arm as bench-only.

## Quick Start

Terminal 1:
```bash
ros2 launch drone_bringup drone.launch.py
```

Terminal 2:
```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

Optional monitoring terminal:
```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/manual_control/send --once
```

## What Changed

- `keyboard_teleop_node` now defaults to `/mavros/manual_control/send`
- primary indoor mode is `ALTCTL`
- `mavros_velocity_node` is not part of the normal props-off teleop path
- pressing `1` or `4` forces manual throttle to zero briefly before arm/disarm is requested
- `SPACE` now means "center sticks", not "emergency stop"

## MAVLink Routing

This setup assumes `mavlink-router` is already running and splitting MAVLink traffic:

- FC -> `mavlink-router`
- `mavlink-router` -> UDP `127.0.0.1:14550` for QGC
- `mavlink-router` -> UDP `127.0.0.1:14540` for MAVROS

QGC and MAVROS can run at the same time. If you want to verify the router process:

```bash
pgrep -af mavlink-router
```

## Pre-Test Checklist

### Hardware

- [ ] Props removed
- [ ] Flight controller connected
- [ ] Battery connected only if needed for motor-output testing
- [ ] Jetson powered on
- [ ] Bench area clear

### Software

- [ ] ROS 2 workspace is sourced
- [ ] `mavlink-router` is running
- [ ] QGC is optional but available for health and mode monitoring
- [ ] QGC virtual joystick is disabled if it interferes with manual control

## Test Procedure

### Phase 1: MAVROS Connection

Start MAVROS:

```bash
ros2 launch drone_bringup drone.launch.py
```

Verify state:

```bash
ros2 topic echo /mavros/state --once
```

Success criteria:

- `connected: true`
- mode is populated
- MAVROS logs show FCU connected

### Phase 2: Keyboard Teleop

Start the teleop node:

```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

The current keymap on the manual backend is:

- `1` arm, with throttle forced low first
- `2` `ALTCTL`
- `3` `STABILIZED`
- `4` disarm, with throttle forced low first
- `0` force-arm, bench only
- `W/S` forward/back
- `A/D` left/right
- `R/F` raise/lower throttle around hover center
- `Q/E` yaw
- `T/Y` adjust stick scaling
- `SPACE` center sticks

Important behavior:

- movement commands latch until another command changes that axis or `SPACE` is pressed
- axes combine, so `R` followed by `W` keeps the raised throttle while adding forward pitch
- on the manual backend, neutral throttle is centered at `z=500`
- during arm/disarm, the node temporarily sends `z=0` so PX4 sees low throttle
- pressing `4` now waits briefly at low throttle before sending the disarm request

### Phase 3: Verify Manual-Control Messages

Check that the node is publishing:

```bash
ros2 topic echo /mavros/manual_control/send --once
```

Expected idle values look roughly like:

- `x: 0`
- `y: 0`
- `z: 500`
- `r: 0`

Optional arm-throttle check:

```bash
timeout 2 ros2 topic echo /mavros/manual_control/send
```

Then press `1` in the teleop window. You should see `z: 0.0` briefly, then it should return to `z: 500.0`.

### Phase 4: Mode Change Test

In the teleop terminal:

1. Press `2` to request `ALTCTL`
2. Confirm the terminal reports the mode change
3. Optionally verify with:

```bash
ros2 topic echo /mavros/state --once
```

Success criteria:

- mode changes to `ALTCTL`
- QGC, if open, reflects `Altitude` / `ALTCTL`

### Phase 5: Arming Test

In the teleop terminal:

1. Press `2` for `ALTCTL`
2. Press `1` to arm

Expected behavior:

- teleop logs that it is holding throttle at zero for the arm request
- FC arms if preflight checks pass
- `/mavros/state` shows `armed: true`

Disarm when done:

1. Press `4`
2. If PX4 still reports `not landed`, press `SPACE`, wait a moment, then press `4` again

If arming fails:

- check QGC for the exact health failure
- confirm `/mavros/manual_control/send` is active
- confirm QGC virtual joystick is not overriding manual control

### Phase 6: Stick Response Test

Only do this with props removed.

Recommended sequence:

1. Arm in `ALTCTL`
2. Press `R` first to raise throttle above center
3. Add one lateral command like `W` or `A`
4. Press `SPACE` after each test to recenter sticks

Suggested checks:

- `R` then `W`: pitch forward with throttle applied
- `R` then `A`: roll left with throttle applied
- `Q`: yaw left command
- `R`: increase throttle above center
- `F`: decrease throttle below center
- `SPACE`: return to neutral sticks

Success criteria:

- motor behavior changes consistently with the requested stick axis
- `SPACE` returns the command to neutral
- no unexpected runaway behavior

### Phase 7: Optional STABILIZED Check

If you want to compare behavior against `ALTCTL`:

1. Press `3` for `STABILIZED`
2. Repeat arm and basic stick checks

Use `ALTCTL` as the primary indoor mode unless you specifically want rawer throttle behavior.

## Post-Test Checklist

- [ ] MAVROS connected cleanly
- [ ] `ALTCTL` mode change worked
- [ ] Arm/disarm path worked with forced-low throttle
- [ ] Manual-control topic published as expected
- [ ] Stick commands changed motor response
- [ ] `SPACE` recenters sticks cleanly
- [ ] No unexplained PX4 health errors remained

## Troubleshooting

### MAVROS does not connect

- verify `mavlink-router` is running
- verify the FC is connected
- check `/mavros/state`
- confirm MAVROS is using `udp://:14540@127.0.0.1:14550`

### Mode change fails

- confirm MAVROS is connected
- verify with `/mavros/state`
- try `ALTCTL` before `STABILIZED`
- inspect QGC for PX4-side rejection reasons

### Arming fails

- check QGC health and arming-failure messages
- confirm the teleop node is publishing `ManualControl`
- confirm the arm request briefly drives throttle to `z=0`
- disable QGC virtual joystick if it is fighting the Jetson input

### Motors do not respond

- confirm the vehicle is actually armed
- confirm you are in `ALTCTL` or `STABILIZED`, not an auto mode
- remember the command latches until you press another key or `SPACE`

### You want to test OFFBOARD instead

That is now a separate bench path:

```bash
ros2 launch drone_bringup bench_offboard.launch.py
```

Use that only for the dummy-vision/OFFBOARD experiments, not for the default indoor teleop workflow.

## Cleanup

Stop the ROS nodes with `Ctrl+C` in each terminal.

If you need logs:

```bash
cd /home/jetson/ros_ws/cdrone_control/ros2/log/latest
ls -la
```
