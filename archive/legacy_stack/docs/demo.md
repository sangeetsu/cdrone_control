# ALTCTL Demo Flight

This document explains how to run the non-invasive automated demo flight that uses the existing
`ALTCTL + MANUAL_CONTROL` path. It does not use the autonomy stack and it does not depend on
`OFFBOARD`.

## What It Does

The demo node performs this sequence after an explicit start command:

1. set `ALTCTL`
2. arm using the same zero-throttle workaround as keyboard teleop
3. take off to a configurable altitude
4. hover
5. move forward a configurable distance
6. hover again
7. switch to `LAND`
8. wait for touchdown
9. disarm

Default values:

- altitude: `0.7 m` (`~2.3 ft`)
- forward distance: `1.2 m` (`~3.9 ft`)

## Important Notes

- This is for the current indoor manual-control path only.
- It depends on `/<mavros_namespace>/manual_control/send`.
- It requires fresh `/<mavros_namespace>/local_position/pose`.
- It will refuse to start if the FCU is disconnected, pose is stale, or the vehicle is already armed.
- If anything goes wrong, use the abort service and be ready to take over manually.

## Preflight

Before running the demo:

- confirm props area is clear and you have a spotter
- confirm MAVLink router is running
- confirm MAVROS connects to the FCU
- confirm `ALTCTL` arming works on this machine
- confirm `local_position/pose` is publishing

Useful checks:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /mavros/manual_control/send --once
```

## Build

```bash
cd /home/sangeetsu/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch drone_bringup altctl_demo.launch.py drone_id:=drone01
```

This launches:

- MAVROS via `drone.launch.py`
- `altctl_demo_sequence_node`

The demo does not auto-start on launch.

## Start The Demo

```bash
ros2 service call /cdrone/drone01/demo/start std_srvs/srv/Trigger "{}"
```

If the service returns `success: false`, read the message. Common reasons:

- FCU not connected
- local pose stale or missing
- mode/arming services unavailable
- vehicle already armed

## Abort The Demo

```bash
ros2 service call /cdrone/drone01/demo/abort std_srvs/srv/Trigger "{}"
```

Abort behavior:

- centers the manual-control command
- requests `LAND`
- leaves the sequence in `ABORT`

## Monitor State

Current demo state:

```bash
ros2 topic echo /cdrone/drone01/demo/state
```

Expected state progression:

```text
IDLE
SET_ALTCTL
ARMING
TAKEOFF
HOVER_AFTER_TAKEOFF
FORWARD_TRANSLATE
HOVER_AFTER_TRANSLATE
SET_LAND
WAIT_TOUCHDOWN
DISARMING
COMPLETE
```

Useful additional monitoring:

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose
ros2 topic echo /mavros/manual_control/send
```

## Parameter Overrides

Example with a 2 ft takeoff and 5 ft forward move:

```bash
ros2 launch drone_bringup altctl_demo.launch.py \
  drone_id:=drone01 \
  target_altitude_m:=0.61 \
  forward_distance_m:=1.52
```

Available launch parameters:

- `drone_id`
- `mavros_namespace`
- `publish_rate_hz`
- `target_altitude_m`
- `forward_distance_m`
- `hover_after_takeoff_s`
- `hover_after_translate_s`
- `arm_zero_throttle_hold_s`
- `takeoff_throttle_delta`
- `hover_throttle_center`
- `forward_stick_cmd`
- `altitude_tolerance_m`
- `position_tolerance_m`
- `stage_timeout_s`
- `land_detect_altitude_m`

## Recommended First Run

Use a conservative first run:

```bash
ros2 launch drone_bringup altctl_demo.launch.py \
  drone_id:=drone01 \
  target_altitude_m:=0.5 \
  forward_distance_m:=0.75 \
  hover_after_takeoff_s:=1.5 \
  hover_after_translate_s:=1.5
```

## Troubleshooting

- If the demo never starts, check `/mavros/state` and `/mavros/local_position/pose`.
- If arming fails, verify the same machine can still arm with the manual teleop path.
- If the vehicle leaves `ALTCTL` unexpectedly during the demo, the node will abort.
- If `LAND` is not accepted on your setup after the forward segment, keep the node structure and
  replace only the landing stage with manual descent logic.

## Related Files

- [`altctl_demo_sequence_node.py`](/home/sangeetsu/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/altctl_demo_sequence_node.py)
- [`altctl_demo.launch.py`](/home/sangeetsu/cdrone_control/ros2/src/drone_bringup/launch/altctl_demo.launch.py)
- [`docs/notes/offboard_indoor_blocker.md`](/home/sangeetsu/cdrone_control/problem.md)
