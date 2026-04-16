#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${ISAAC_ROS_CONTAINER_NAME:-isaac_ros_dev-aarch64-container}"
LOG_PATH="${ISAAC_VSLAM_LOG_PATH:-/tmp/isaac_vslam_launch.log}"
POST_FIX_LOG_PATH="${ISAAC_VSLAM_POST_FIX_LOG_PATH:-/tmp/isaac_vslam_post_fix.log}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is not running." >&2
  echo "Start it with ./scripts/start_isaac_realsense_container.sh first." >&2
  exit 1
fi

"${REPO_ROOT}/scripts/ensure_isaac_vslam_deps_in_container.sh"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "pkill -9 -f '^/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node( |$)' || true; \
   pkill -9 -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py( |$)' || true; \
   pkill -9 -f '^/opt/ros/humble/lib/rclcpp_components/component_container(.*)__node:=visual_slam_launch_container( |$)' || true"

docker exec -d -u admin -w /workspaces/isaac_ros-dev "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py \
   enable_color:=true enable_depth:=true \ 
> ${LOG_PATH} 2>&1"

# NVIDIA documents that the D455 IR stream on Jetson can start capped around
# 15 FPS. On this Jetson the reliable workaround is to re-apply the RealSense
# auto-exposure parameter after the node is up, then apply it a second time
# after a short delay.
docker exec -d -u admin -w /workspaces/isaac_ros-dev "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   { for _ in \$(seq 1 30); do \
       result=\$(ros2 param set /camera/camera depth_module.enable_auto_exposure true 2>&1 || true); \
       echo \"\${result}\"; \
       if echo \"\${result}\" | grep -Fq 'Set parameter successful'; then \
         sleep 5; \
         result=\$(ros2 param set /camera/camera depth_module.enable_auto_exposure true 2>&1 || true); \
         echo \"\${result}\"; \
         if echo \"\${result}\" | grep -Fq 'Set parameter successful'; then \
           exit 0; \
         fi; \
       fi; \
       sleep 1; \
     done; \
     echo 'Failed to re-apply depth_module.enable_auto_exposure for /camera/camera' >&2; \
     exit 1; \
   } > ${POST_FIX_LOG_PATH} 2>&1"

echo "Launched Isaac ROS Visual SLAM with the D455 inside ${CONTAINER_NAME}."
echo "Launch log: ${LOG_PATH}"
echo "Post-fix log: ${POST_FIX_LOG_PATH}"
