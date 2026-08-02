# cdrone4 Simple Hover Test Runbook

This runbook is for a supervised `cdrone4` / `RigidBody4` hover test using
the existing `position_hover_demo` sequence.

The default test profile is:

- drone id: `cdrone4`
- rigid body: `RigidBody4`
- source pose: `/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose`
- MAVROS namespace: `/cdrone/cdrone4/mavros`
- OptiTrack server: `192.168.0.217`
- external-pose frame rotation: `[0.0, 0.0, 3.141592653589793]`
- takeoff altitude: `1.0 m`
- hover duration: `10 s`
- speed profile: disabled

The run script does not start flight. It launches the stack, starts logging, and
waits for readiness. The operator starts the hover with a separate command.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_control_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Preflight

Use this only with a clear test area, a charged battery, props secured, QGC/RC
available, and a person ready to stop the vehicle.

The run script starts MAVLink routing automatically if `mavlink-routerd` is not
already running. To start it manually first:

```bash
/home/jetson/.local/bin/start_mavlink_router.sh
```

Confirm Motive is streaming `RigidBody4`, then use the cdrone4 launch path. Do
not edit `ros2/src/drone_bringup/config/droneid_config.yaml`; the scripts pass
the cdrone4 values explicitly.

## Launch And Capture

Terminal 1:

```bash
cd /home/jetson/cdrone_control
scripts/run_simple_hover_test.sh
```

The script restores the hover baseline PX4 parameters, launches
`position_hover_demo.launch.py`, starts `capture_position_hover_debug.sh`, and
waits for readiness. It prints the exact start and abort commands when ready.
If the baseline restore cannot reach PX4, check:

- `mavlink-routerd` is running
- `/dev/serial/by-id/*ARK*` exists
- the FC is powered and connected over USB
- `flight_logs/simple_hover/<run_id>/mavlink_router_start.log`
- `flight_logs/simple_hover/<run_id>/baseline_restore.log`

If the baseline is already known-good and you only want to launch the stack:

```bash
scripts/run_simple_hover_test.sh --skip-baseline-restore
```

Artifacts are written under:

```text
flight_logs/simple_hover/<run_id>/
flight_logs/simple_hover/<run_id>/capture/summary.txt
```

## Start

Terminal 2, only after Terminal 1 says ready:

```bash
cd /home/jetson/cdrone_control
scripts/start_simple_hover_test.sh
```

Expected state path:

```text
IDLE
SYNC_TAKEOFF_PARAM
SET_TAKEOFF_MODE
ARMING
TAKEOFF
SET_HOVER_MODE
HOVER
SET_LAND_MODE
WAIT_TOUCHDOWN
DISARMING
RESTORE_TAKEOFF_PARAM
COMPLETE
```

Some states may pass quickly or be skipped depending on PX4 handoff timing. A
successful run ends at `COMPLETE`, landed and disarmed.

## Abort

Normal abort command:

```bash
cd /home/jetson/cdrone_control
scripts/abort_simple_hover_test.sh
```

Use abort immediately if:

- OptiTrack tracking is unstable or drops
- MAVROS disconnects
- the drone drifts horizontally more than expected
- the drone climbs, descends, or lands in a way you are not comfortable with
- the demo state reaches `ABORT`

The hover node also aborts automatically on stale local pose, stale MAVROS
state, inactive external-pose companion status, or horizontal excursion beyond
`0.75 m`.

## Force Kill

Use this only after the vehicle is safe on the ground or when a separate manual
landing/disarm path is already in control. Force killing software does not send
a landing command.

First try an interrupt cleanup:

```bash
pkill -INT -f 'run_simple_hover_test.sh|capture_position_hover_debug.sh|ros2 bag record|ros2 launch drone_bringup position_hover_demo.launch.py|position_hover_demo_sequence_node'
```

If those processes are still wedged after a few seconds, force kill them:

```bash
pkill -9 -f 'run_simple_hover_test.sh|capture_position_hover_debug.sh|ros2 bag record|ros2 launch drone_bringup position_hover_demo.launch.py|position_hover_demo_sequence_node'
```

After a force kill, verify the vehicle is disarmed and rerun the baseline restore
before the next attempt:

```bash
/home/jetson/cdrone_control/scripts/restore_hover_baseline_params.sh
```

## Review Artifacts

After the run, inspect:

```bash
RUN_DIR="$(find /home/jetson/cdrone_control/flight_logs/simple_hover -maxdepth 1 -type d -name 'simple_hover_cdrone4_*' | sort | tail -n 1)"
sed -n '1,220p' "${RUN_DIR}/capture/summary.txt"
tail -n 80 "${RUN_DIR}/position_hover_launch.log"
```

The summary should show samples for demo state, MAVROS state, local pose, vision
pose, companion status, and recent state transitions ending in `COMPLETE`.
