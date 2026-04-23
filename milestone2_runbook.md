# Milestone 2 Runbook

Commands below assume your shell already sourced ROS 2 and the workspace from
`~/.bashrc`. Detailed notes, status, and validation context live in
`docs/milestones/milestone2.md`.

The active `drone_id`, MAVROS namespace, and mocap defaults come from
`ros2/src/drone_bringup/config/droneid_config.yaml`. The checked-in commands use
the current `cdrone4` branch values.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
```

## Launch

```bash
MODEL_PATH=/home/jetson/cdrone_control/models/16_k_and_drone_studio_realsense_images_model.engine
ros2 launch drone_bringup milestone2_demo.launch.py \
  model_path:="$MODEL_PATH" \
  required_completion_count:=1
```

## Start

Run this after supervised takeoff and staging hover:

```bash
ros2 service call /cdrone/cdrone4/demo/milestone2_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/cdrone4/demo/milestone2_abort std_srvs/srv/Trigger "{}"
```
