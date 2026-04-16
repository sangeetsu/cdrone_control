# Milestone 1 Runbook

This runbook is for the first flight-test pass of:

- `position_hover_demo`
- `position_goto_demo`
- `position_circle_demo`

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
- you are flying with the current local `map` frame from `/mavros/local_position/pose`

If your active flight rigid body is not the default one, add:

```bash
rigid_body_name:=RigidBody3
```

or replace `RigidBody3` with the correct body name in the launch commands below.

## One-Time Build

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/install/setup.bash
```

## Terminal Setup

Use three terminals.

Terminal 1: launch the active demo

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/install/setup.bash
```

Terminal 2: watch demo state

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/install/setup.bash
```

Terminal 3: send start and abort service calls

```bash
cd /home/jetson/cdrone_control
source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/install/setup.bash
```

## Preflight Checks

Run these before each flight attempt.

```bash
ros2 topic echo --once /mavros/state
ros2 topic echo --once /mavros/local_position/pose
ros2 topic echo --once /mavros/companion_process/status
ros2 topic hz /mavros/local_position/pose
```

What you want to see:

- `/mavros/state` is updating and connected
- `/mavros/local_position/pose` is updating cleanly
- companion status is present
- local pose rate is healthy

## Phase 1: Hover Demo

Terminal 1:

```bash
ros2 launch drone_bringup position_hover_demo.launch.py \
  takeoff_altitude_m:=1.5 \
  hover_duration_s:=30.0 \
  use_speed_profile:=true
```

Terminal 2:

```bash
ros2 topic echo /cdrone/drone01/demo/position_hover_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/drone01/demo/position_hover_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/drone01/demo/position_hover_abort std_srvs/srv/Trigger "{}"
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
  use_speed_profile:=true \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/drone01/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_abort std_srvs/srv/Trigger "{}"
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

This point is south of the pillar keep-out, stays comfortably inside the studio boundary, and lines up with the current circle demo geometry. Keep the indoor speed profile enabled for this phase.

Terminal 1:

```bash
ros2 launch drone_bringup position_goto_demo.launch.py \
  goal_x_m:=-5.62 \
  goal_y_m:=0.40 \
  goal_z_m:=2.0 \
  max_goal_distance_from_start_m:=8.0 \
  use_speed_profile:=true \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/drone01/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_abort std_srvs/srv/Trigger "{}"
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
  use_speed_profile:=true \
  perimeter_boundary_margin_m:=1.0 \
  perimeter_keep_out_margin_m:=0.3
```

Terminal 2:

```bash
ros2 topic echo /cdrone/drone01/demo/position_goto_state
```

Terminal 3:

Start:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_start std_srvs/srv/Trigger "{}"
```

Abort if needed:

```bash
ros2 service call /cdrone/drone01/demo/position_goto_abort std_srvs/srv/Trigger "{}"
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
