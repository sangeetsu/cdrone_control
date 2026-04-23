# Milestone 3 Runbook

Commands below assume your shell already loaded ROS 2 and the local workspace.
Detailed notes live in `docs/milestones/milestone3.md`.

The active `drone_id`, MAVROS namespace, and mocap defaults come from
`ros2/src/drone_bringup/config/droneid_config.yaml`. The checked-in commands use
the current `cdrone4` branch values.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
source /home/jetson/cdrone_control/ros2/install/setup.bash
```

## Launch

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1
```

PX4 speed profile selection for Milestone 3:

- `speed_profile:=indoor` for the current conservative profile
- `speed_profile:=fun` for the new midpoint profile
- `speed_profile:=default` for the faster baseline profile

Example:

```bash
ros2 launch drone_bringup milestone3_demo.launch.py \
  required_completion_count:=1 \
  speed_profile:=fun
```

## Start

```bash
ros2 service call /cdrone/cdrone4/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/cdrone4/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```

## Speed Tuning

PX4 post-OFFBOARD profiles live in
`ros2/src/drone_bringup/config/indoor_speed_profile.yaml`,
`fun_speed_profile.yaml`, and `default_speed_profile.yaml`. Controller-side
follow and yaw response still comes from
`ros2/src/drone_bringup/config/milestone3_demo.yaml`. The detailed values,
units, and tuning guidance live in `docs/milestones/milestone3.md`.
