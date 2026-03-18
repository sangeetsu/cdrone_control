# cdrone_control

ROS 2 control and autonomy stack for a Jetson Orin Nano multicopter running PX4 via MAVROS.

## Current Status

This repo now has two distinct control paths:

- `ALTCTL` / `STABILIZED` manual teleop from the Jetson keyboard through MAVROS `ManualControl`
- velocity-based `OFFBOARD` plumbing for autonomy and bench experiments

What is currently proven:

- `ros2 launch drone_bringup drone.launch.py` brings up MAVROS against PX4 over `mavlink-router`
- `ros2 run drone_control_pkg keyboard_teleop_node` works for indoor and outdoor manual teleop in `ALTCTL`
- keyboard arm/disarm now forces throttle low first
- keyboard axes now combine correctly, so throttle and pitch/roll can be commanded together
- normal bench `OFFBOARD` mode can be reached with the dummy vision publisher in `bench_offboard.launch.py`
- a first in-repo stereo VIO path now exists for indoor external-vision bringup through `/mavros/vision_pose/pose`

What is not ready yet:

- no real no-GPS `OFFBOARD` autonomy without external vision / VIO or optical flow
- the current autonomy path still depends on `mavros_velocity_node` and `OFFBOARD`
- the dummy external-vision node is bench-only and not flightworthy
- the new stereo VIO path still needs calibration, extrinsic tuning, and PX4 EKF validation before flight

## Important Learnings

1. This is a PX4 stack, not an ArduPilot GUIDED stack.
2. Indoor teleop is currently best done in `ALTCTL`, not `OFFBOARD`.
3. `keyboard_teleop_node` now publishes to `/mavros/manual_control/send` by default.
4. `SPACE` means "center sticks", not emergency stop.
5. QGC can coexist with MAVROS through `mavlink-router`, but if QGC shows transfer timeouts or MAVROS becomes unstable, close QGC and disable QGC virtual joystick.
6. `OFFBOARD` without GPS is not the real issue. The actual requirement is a trusted aiding source such as VIO / external vision or optical flow + rangefinder.

## Key Docs

- [props_off_test.md](./props_off_test.md): current indoor / bench procedure
- [props_on_test.md](./props_on_test.md): first low-altitude outdoor flight procedure
- [problem.md](./problem.md): current `OFFBOARD` diagnosis and status
- [vio_todo.md](./vio_todo.md): stereo-to-VIO follow-up work
- [docs/indoor_vio_px4.md](./docs/indoor_vio_px4.md): step-by-step indoor VIO bringup

## Platform Assumptions

- Jetson Orin Nano 8 GB
- ROS 2 Humble
- PX4 1.16 on ARKV6X
- MAVROS over UDP through `mavlink-router`
- FCU URL: `udp://:14540@127.0.0.1:14550`
- QGC, if used, talks to `127.0.0.1:14550`

## Build

```bash
cd /home/jetson/ros_ws/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Current Manual Teleop Path

Launch MAVROS:

```bash
ros2 launch drone_bringup drone.launch.py
```

Run keyboard teleop:

```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

Current manual backend behavior:

- key `2`: `ALTCTL`
- key `3`: `STABILIZED`
- key `1`: arm with throttle forced low first
- key `4`: disarm after a short low-throttle delay
- `W/A/S/D/Q/E/R/F` latch and combine across axes until changed or `SPACE`
- `SPACE` recenters all sticks

Recommended first mode:

- use `ALTCTL` for manual flight tests
- use `STABILIZED` only if you specifically want more raw throttle behavior

## Current OFFBOARD Bench Path

Bench-only `OFFBOARD` launch:

```bash
ros2 launch drone_bringup bench_offboard.launch.py
```

This path starts:

- MAVROS
- the dummy external-vision pose publisher
- the velocity bridge for `OFFBOARD` setpoints

Use this only to validate control-path plumbing. It is not the flight-ready no-GPS autonomy solution.

## Indoor VIO Path

For indoor no-GPS position aiding, use:

```bash
ros2 launch drone_bringup indoor_vio.launch.py
```

This starts:

- MAVROS
- `stereo_vio_node`
- `px4_vision_bridge_node`

Calibration is expected at:

```bash
/home/jetson/cdrone_control/calibration/stereo_calibration.npz
```

See [docs/indoor_vio_px4.md](./docs/indoor_vio_px4.md) for the full bringup sequence.

## Full Autonomy Stack

The autonomy stack still uses the velocity-`OFFBOARD` path:

```bash
ros2 launch drone_bringup autonomy_stack.launch.py
```

At a high level:

- `stereo_tracker_node` publishes perception tracks
- `engagement_manager_node` publishes `/cdrone/control/cmd_vel_body`
- `mavros_velocity_node` gates and forwards velocity commands to `/mavros/setpoint_velocity/cmd_vel`

Do not rely on this path for no-GPS indoor flight until a real VIO / external-vision estimator is integrated.

## QGC and MAVROS

This repo expects `mavlink-router` to split traffic between MAVROS and QGC.

Expected pattern:

- FC -> `mavlink-router`
- `mavlink-router` -> `127.0.0.1:14550` for QGC
- `mavlink-router` -> `127.0.0.1:14540` for MAVROS

Notes:

- QGC is useful for health and arming diagnostics
- QGC virtual joystick must stay disabled when using Jetson keyboard teleop
- if QGC reports transfer timeouts or MAVROS becomes unstable, close QGC before flight testing

## Safety Guidance

- Props-off testing first, always
- First props-on flights should be manual `ALTCTL`, outdoors, low altitude, and not `OFFBOARD`
- Do not attempt no-GPS autonomous flight until a real aiding source exists
- Keep a spotter and a clear landing plan for first props-on tests

## Repository Layout

```text
cdrone_control/
|- problem.md
|- props_off_test.md
|- props_on_test.md
|- vio_todo.md
|- ros2/
|  |- src/
|  |  |- drone_bringup/
|  |  |- drone_msgs/
|  |  |- drone_vision_pkg/
|  |  |- drone_behavior_pkg/
|  |  |- drone_control_pkg/
|  |  |- drone_light_pkg/
|  |  |- ros2_poselib/
|- requirements-jetson.txt
|- requirements-dev.txt
```

## Main Package Roles

- `drone_bringup`: launch files and MAVROS config
- `drone_control_pkg`: keyboard teleop, MAVROS adapters, bench vision publisher
- `drone_behavior_pkg`: autonomy state machine and health monitoring
- `drone_vision_pkg`: stereo perception and tracking
- `drone_light_pkg`: spotlight hardware control
- `drone_msgs`: shared ROS message types

## Useful Commands

Check MAVROS state:

```bash
ros2 topic echo /mavros/state --once
```

Check manual-control messages:

```bash
ros2 topic echo /mavros/manual_control/send --once
```

Check velocity setpoints for the autonomy path:

```bash
ros2 topic hz /mavros/setpoint_velocity/cmd_vel
```

Syntax check Python nodes:

```bash
python3 -m py_compile \
  ros2/src/drone_behavior_pkg/drone_behavior_pkg/*.py \
  ros2/src/drone_light_pkg/drone_light_pkg/*.py \
  ros2/src/drone_vision_pkg/drone_vision_pkg/*.py \
  ros2/src/drone_control_pkg/drone_control_pkg/*.py
```
