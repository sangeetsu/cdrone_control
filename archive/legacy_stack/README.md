# Legacy Stack Archive

This directory holds the parts of `cdrone_control` that were removed from the active workspace during the VIO-first rehaul.

## Why It Exists

The old repo mixed:

- manual MAVROS teleop
- target-tracking autonomy
- lighting control
- monitoring/dashboard code
- early external-vision experiments

That made it harder to focus the active repo on the actual blocker: D455 VIO into PX4 for indoor position control.

## What Was Archived

- `ros2/src/drone_behavior_pkg`
- `ros2/src/drone_vision_pkg`
- `ros2/src/drone_light_pkg`
- `ros2/src/drone_monitor_pkg`
- legacy launch files from `drone_bringup`
- legacy plans, setup notes, and test procedures
- the old Jetson autonomy container files

## How To Use It

Treat this archive as reference material:

- recover older implementation ideas
- recover old tests or procedures
- mine code during the rehaul

Do not treat it as the active source of truth for the current repo layout.
