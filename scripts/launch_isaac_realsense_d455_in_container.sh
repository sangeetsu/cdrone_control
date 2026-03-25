#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${ISAAC_ROS_CONTAINER_NAME:-isaac_ros_dev-aarch64-container}"
LOG_PATH="${ISAAC_REALSENSE_LOG_PATH:-/tmp/isaac_realsense_launch.log}"
POST_FIX_LOG_PATH="${ISAAC_REALSENSE_POST_FIX_LOG_PATH:-/tmp/isaac_realsense_post_fix.log}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is not running." >&2
  echo "Start it with ./scripts/start_isaac_realsense_container.sh first." >&2
  exit 1
fi

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "pkill -9 -f '^/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node( |$)' || true; \
   pkill -9 -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch realsense2_camera rs_launch.py( |$)' || true"

# This is the best current launch profile for the repo's VIO path on this
# Jetson. The extra toggles did not fully clear the remaining control-transfer
# errors, but they map directly to the failure modes seen during validation.
docker exec -d -u admin -w /workspaces/isaac_ros-dev "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 launch realsense2_camera rs_launch.py \
     camera_name:=d455 \
     enable_color:=false \
     enable_depth:=false \
     enable_infra1:=true \
     enable_infra2:=true \
     enable_gyro:=true \
     enable_accel:=true \
     unite_imu_method:=2 \
     depth_module.profile:=640x360x90 \
     gyro_fps:=200 \
     accel_fps:=200 \
     pointcloud.enable:=false \
     align_depth.enable:=false \
     publish_tf:=true \
     depth_module.global_time_enabled:=false \
     motion_module.global_time_enabled:=false \
     depth_module.thermal_compensation:=false \
     hold_back_imu_for_frames:=false \
     infra1_qos:=SENSOR_DATA \
     infra2_qos:=SENSOR_DATA \
     gyro_qos:=SENSOR_DATA \
     accel_qos:=SENSOR_DATA \
     > ${LOG_PATH} 2>&1"

# NVIDIA documents a D455-on-Jetson issue where the IR stream can start capped
# at 15 FPS even when 90 FPS is requested. Re-applying this runtime parameter
# after the node comes up consistently restores the expected IR rate here.
docker exec -d -u admin -w /workspaces/isaac_ros-dev "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   { for _ in \$(seq 1 30); do \
       result=\$(ros2 param set /d455/d455 depth_module.enable_auto_exposure true 2>&1 || true); \
       echo \"\${result}\"; \
       if echo \"\${result}\" | grep -Fq 'Set parameter successful'; then \
         sleep 5; \
         result=\$(ros2 param set /d455/d455 depth_module.enable_auto_exposure true 2>&1 || true); \
         echo \"\${result}\"; \
         if echo \"\${result}\" | grep -Fq 'Set parameter successful'; then \
           exit 0; \
         fi; \
       fi; \
       sleep 1; \
     done; \
     echo 'Failed to re-apply depth_module.enable_auto_exposure for /d455/d455' >&2; \
     exit 1; \
   } > ${POST_FIX_LOG_PATH} 2>&1"

echo "Launched realsense2_camera inside ${CONTAINER_NAME}."
echo "Log: ${LOG_PATH}"
echo "Tail it with: docker exec -u admin ${CONTAINER_NAME} tail -n 120 ${LOG_PATH}"
echo "Post-fix log: ${POST_FIX_LOG_PATH}"
