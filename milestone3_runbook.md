# Milestone 3 Runbook

Commands below assume your shell already sourced ROS 2 and the workspace from
`~/.bashrc`. Detailed notes live in `milestone3.md`.

## Build

```bash
cd /home/jetson/cdrone_control/ros2
colcon build --packages-select drone_msgs drone_control_pkg drone_vision_pkg drone_bringup
```

## Launch

```bash
MODEL_PATH=/home/jetson/cdrone_control/models/16_k_and_drone_studio_realsense_images_model.engine
ros2 launch drone_bringup milestone3_demo.launch.py \
  model_path:="$MODEL_PATH" \
  required_completion_count:=1
```

## Start

```bash
ros2 service call /cdrone/drone01/demo/milestone3_start std_srvs/srv/Trigger "{}"
```

## Abort

```bash
ros2 service call /cdrone/drone01/demo/milestone3_abort std_srvs/srv/Trigger "{}"
```
