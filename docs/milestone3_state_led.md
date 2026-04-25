# Milestone 3 State Topic and blink(1) LED

The Milestone 3 demo state topic is:

```bash
/cdrone/cdrone3/engagement/state
```

The generic form is:

```bash
/cdrone/<drone_id>/engagement/state
```

For this repo, `ros2/src/drone_bringup/config/droneid_config.yaml` sets
`drone_id: cdrone3`, so the active topic is
`/cdrone/cdrone3/engagement/state`.

The message type is `drone_msgs/msg/EngagementState`. The field that carries the
Milestone 3 behavior state is `state`.

Useful checks:

```bash
ros2 topic echo --once /cdrone/cdrone3/engagement/state
ros2 interface show drone_msgs/msg/EngagementState
```

The Milestone 3 node publishes this topic from
`milestone3_demo_sequence_node.py` through the `engagement_state_topic`
parameter. If that launch argument is left empty, the node defaults to
`/cdrone/<drone_id>/engagement/state`.

## State Names

The current Milestone 3 state machine includes:

- `IDLE`
- `SYNC_TAKEOFF_PARAM`
- `SYNC_PRE_TAKEOFF_PROFILE`
- `SET_TAKEOFF_MODE`
- `ARMING`
- `REQUEST_TAKEOFF`
- `TAKEOFF`
- `WARMUP_OFFBOARD`
- `SET_OFFBOARD_MODE`
- `SYNC_SPEED_PROFILE`
- `STAGE_HOVER`
- `SEARCH`
- `FOLLOW`
- `DWELL`
- `RETURN_TO_START`
- `RETURN_HOME_HOLD`
- `SET_LAND_MODE`
- `WAIT_TOUCHDOWN`
- `DISARMING`
- `RESTORE_TAKEOFF_PARAM`
- `RESTORE_SPEED_PROFILE`
- `COMPLETE`
- `ABORT`

## LED Script

`scripts/led_control.py` subscribes to
`/cdrone/cdrone3/engagement/state` and maps `EngagementState.state` to a ThingM
blink(1) color:

- off: `IDLE`
- blue: startup, parameter sync, arming, takeoff, offboard setup, and stage hover
- yellow: `SEARCH`
- green: `FOLLOW`
- cyan: `DWELL`
- violet: `RETURN_TO_START` or `RETURN_HOME_HOLD`
- amber: landing, disarming, and cleanup states
- white: `COMPLETE`
- red: `ABORT` or latched estop
- orange-red: any nonterminal state with a nonempty `blocked_reason`
- gray: unknown state

Run it after sourcing ROS and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ros2/install/setup.bash
python3 scripts/led_control.py
```

Override the topic if needed:

```bash
python3 scripts/led_control.py --state-topic /cdrone/cdrone4/engagement/state
```

Bench checks:

```bash
python3 scripts/led_control.py --dry-run
python3 scripts/led_control.py --test-color green
```

## blink(1) Permission Fix

The non-sudo failure reproduced as:

```text
blink1.blink1.Blink1ConnectionFailed: open failed
```

The local blink(1) is USB `27b8:01ed`. Its active HID node was owned by
`root:root`, so the `jetson` user could read but not open it for writing. Install
the udev rule once:

```bash
sudo scripts/install_blink1_host_access.sh
```

Then unplug and replug the blink(1). The rule lives at
`scripts/blink1/99-blink1.rules` and grants normal-user write access to the
blink(1) HID node.
