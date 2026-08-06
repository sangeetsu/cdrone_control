# Position Hover Demo Runbook

Use this for `cdrone3` indoor OptiTrack hover checks. Keep props/ESC safety appropriate for the phase. Do not call the start service until the pose chain is healthy and the flight area is clear.

## Bridge-Only Pose Check

This launch is for checking MAVROS, VRPN, and external-vision pose flow only. It does not start the hover demo node, so it will not create the `position_hover_start` or `position_hover_abort` services.

```bash
source ~/.bashrc
ros2 launch drone_bringup external_pose_px4_bridge.launch.py \
  enable_reference_setup:=false \
  enable_pose_debug:=true \
  rigid_body_name:=RigidBody4
```

Expected checks:

```bash
ros2 topic hz /cdrone/cdrone3/vrpn_mocap/RigidBody4/pose
ros2 topic hz /cdrone/cdrone3/external_pose/input_pose
ros2 topic hz /cdrone/cdrone3/mavros/vision_pose/pose
ros2 topic hz /cdrone/cdrone3/mavros/local_position/pose
ros2 topic echo --once /cdrone/cdrone3/mavros/estimator_status
ros2 topic echo --once /cdrone/cdrone3/mavros/statustext/recv
```

Good signs:

- `RigidBody4` and `/external_pose/input_pose` publish around 40 Hz or higher.
- `/mavros/vision_pose/pose` publishes around 30 Hz.
- `/mavros/local_position/pose` publishes around 10 Hz.
- Stationary local pose does not walk in `x/y`.
- No active preflight error remains after the external-vision yaw settles.

Stop this launch before starting a separate hover-demo launch.

## Hover Demo Launch

The start and abort services are created only by `position_hover_demo.launch.py`.

```bash
source ~/.bashrc
ros2 launch drone_bringup position_hover_demo.launch.py \
  pose_source:=optitrack \
  optitrack_server:=192.168.0.217 \
  rigid_body_name:=RigidBody4 \
  use_speed_profile:=false \
  takeoff_altitude_m:=1.0 \
  takeoff_rate_m_s:=0.3 \
  takeoff_strategy:=AUTO_MODE \
  hover_duration_s:=10.0 \
  hover_mode:=HOLD \
  max_horizontal_excursion_m:=0.5
```

Confirm the services exist:

```bash
ros2 service list | grep position_hover
```

Expected services:

```text
/cdrone/cdrone3/demo/position_hover_abort
/cdrone/cdrone3/demo/position_hover_start
```

Monitor state and PX4 messages from other terminals:

```bash
ros2 topic echo /cdrone/cdrone3/demo/position_hover_state
ros2 topic echo /cdrone/cdrone3/mavros/state
ros2 topic echo /cdrone/cdrone3/mavros/local_position/pose
ros2 topic echo /cdrone/cdrone3/mavros/statustext/recv
```

## Start And Abort

Start hover demo:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

Abort hover demo:

```bash
ros2 service call /cdrone/cdrone3/demo/position_hover_abort std_srvs/srv/Trigger "{}"
```

## Disarm And Emergency Commands

Normal disarm:

```bash
ros2 service call /cdrone/cdrone3/mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

Force disarm / motor kill:

```bash
ros2 service call /cdrone/cdrone3/mavros/cmd/command mavros_msgs/srv/CommandLong "{broadcast: false, command: 400, confirmation: 0, param1: 0.0, param2: 21196.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

Emergency flight termination, if enabled in PX4:

```bash
ros2 service call /cdrone/cdrone3/mavros/cmd/command mavros_msgs/srv/CommandLong "{broadcast: false, command: 185, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
```

Use force disarm or emergency termination only as an emergency action. They can stop motors immediately.

## Notes

- If the start service is missing, the hover demo node is not running. The bridge-only launch is not enough.
- Current mocap evidence showed `RigidBody4` publishing for this drone; `RigidBody3` produced no source pose samples.
- The `vrpn_mocap` client may print an exit-code `-11` error during Ctrl-C shutdown. Treat that as a shutdown cleanup issue unless it happens during active operation.
