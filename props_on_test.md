# Props-On Flight Test Guide

**Scope:** first low-altitude manual flight with propellers installed.

This guide is for the current working path in this repo:

- MAVROS bringup with `drone.launch.py`
- Jetson keyboard teleop with `keyboard_teleop_node`
- manual `ALTCTL` flight only

This guide is **not** for:

- `OFFBOARD` flight
- indoor flight
- no-GPS autonomy
- dummy-vision bench tests

## Safety Rules

- Fly outdoors only, in a wide open area.
- Keep clear of people, vehicles, buildings, trees, and power lines.
- Use a spotter if possible.
- Keep the first flight low and short.
- Do not leave QGC virtual joystick enabled.
- If QGC starts reporting transfer timeouts or MAVROS becomes unstable, land and close QGC before the next attempt.

## Before You Do This

Complete [props_off_test.md](./props_off_test.md) successfully first.

Do not proceed until all of the following are true:

- the props-off bench test passes
- arming works from `keyboard_teleop_node`
- `ALTCTL` mode change works
- `SPACE` recenters sticks as expected
- you understand that the keyboard commands latch until changed or `SPACE` is pressed

## Hardware Checklist

- [ ] Propellers installed in the correct rotation and orientation
- [ ] Props tightened correctly
- [ ] Battery charged and secured
- [ ] Flight controller mounted securely
- [ ] GPS is optional for this manual `ALTCTL` test, but all health checks must pass
- [ ] Launch area is clear

## Software Checklist

- [ ] ROS 2 workspace built and sourced
- [ ] `mavlink-router` running
- [ ] QGC virtual joystick disabled if QGC is open
- [ ] MAVROS connects cleanly

## Launch

Terminal 1:

```bash
ros2 launch drone_bringup drone.launch.py
```

Terminal 2:

```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

Optional monitoring:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/manual_control/send --once
```

## Mode and Arming Sequence

In the teleop terminal:

1. Press `2` for `ALTCTL`
2. Confirm the mode change
3. Press `1` to arm

Expected behavior:

- throttle is forced low before the arm request
- PX4 arms if health checks pass
- the vehicle stays armed until takeoff or disarm

If arming fails:

- inspect QGC health messages
- confirm `/mavros/manual_control/send` is publishing
- confirm QGC virtual joystick is not interfering

## First Takeoff

This keyboard interface is coarse. Treat it like a minimal flight test, not a normal piloting setup.

Recommended first lift:

1. After arming, press `R`
2. Watch for lift-off
3. As soon as the vehicle reaches about 0.5-1.0 m altitude, press `SPACE`

Why this works:

- `R` raises throttle above center
- `SPACE` returns throttle to center, which is the `ALTCTL` hold region

Keep the first hover:

- below about 1-2 m
- close to the takeoff point
- very short

## Basic Control Test

Only make one small correction at a time.

Recommended pattern:

1. Press one correction key
2. Watch the response
3. Press `SPACE` to re-center

Useful corrections:

- `W` / `S`: pitch forward / back
- `A` / `D`: roll left / right
- `Q` / `E`: yaw left / right
- `R` / `F`: climb / descend
- `SPACE`: re-center all sticks

Important:

- commands latch until changed or `SPACE` is pressed
- a short tap is still a sustained command until you cancel it

## Landing

Recommended landing sequence:

1. Press `F` to begin descending
2. Press `SPACE` shortly before touchdown or immediately after touchdown
3. Wait briefly for PX4 to detect landed
4. Press `4` to disarm

If PX4 says `Disarming denied: not landed`:

1. Press `SPACE`
2. Wait 1-2 seconds
3. Press `4` again

## Abort / Stop Criteria

End the test immediately if any of the following happens:

- vehicle drifts more than you expected
- vehicle climbs higher than planned
- oscillation starts
- MAVROS logs start showing connection trouble
- QGC reports transfer errors and the system feels unstable

Immediate recovery actions:

- press `SPACE`
- if too high, press `F` and then `SPACE`
- land and disarm

## What Not To Do Yet

- do not use `OFFBOARD` with props on from this repo yet
- do not use the dummy vision node for real flight
- do not attempt indoor autonomous flight without a real VIO / external-vision estimator
- do not use the first props-on test to validate autonomy

## After Flight

- inspect MAVROS logs for errors
- note any arming, landing, or disarm oddities
- review the PX4 ULog if needed

## References

- PX4 v1.16 arm/disarm config: https://docs.px4.io/v1.16/en/advanced_config/prearm_arm_disarm
- PX4 v1.16 altitude mode: https://docs.px4.io/v1.16/en/flight_modes_mc/altitude.html
- PX4 first flight guidelines: https://docs.px4.io/v1.16/en/flying/first_flight_guidelines.html
