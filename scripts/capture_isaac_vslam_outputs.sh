#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${ISAAC_ROS_CONTAINER_NAME:-isaac_ros_dev-aarch64-container}"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT_DIR="${ISAAC_VSLAM_OUTPUT_DIR:-${REPO_ROOT}/temp_outputs/vio_${STAMP}}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is not running." >&2
  echo "Start it and launch VSLAM first." >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"

docker exec -u admin "${CONTAINER_NAME}" rs-enumerate-devices \
  > "${OUTPUT_DIR}/realsense_device.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 node list | sort" \
  > "${OUTPUT_DIR}/ros_nodes.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic list | sort" \
  > "${OUTPUT_DIR}/ros_topics.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && ros2 topic info /visual_slam/tracking/odometry" \
  > "${OUTPUT_DIR}/visual_slam_odometry_info.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 12 ros2 topic echo --once /visual_slam/status" \
  > "${OUTPUT_DIR}/visual_slam_status_once.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 12 ros2 topic echo --once /visual_slam/tracking/odometry" \
  > "${OUTPUT_DIR}/visual_slam_odometry_once.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 12 ros2 topic echo --once /visual_slam/tracking/vo_pose_covariance" \
  > "${OUTPUT_DIR}/visual_slam_pose_once.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /visual_slam/tracking/odometry" \
  > "${OUTPUT_DIR}/visual_slam_odometry_hz.txt" || true

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /camera/infra1/image_rect_raw" \
  > "${OUTPUT_DIR}/camera_infra1_hz.txt" || true

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /camera/imu" \
  > "${OUTPUT_DIR}/camera_imu_hz.txt" || true

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "tail -n 200 /tmp/isaac_vslam_launch.log" \
  > "${OUTPUT_DIR}/isaac_vslam_launch_tail.txt"

docker exec -u admin "${CONTAINER_NAME}" /bin/bash -lc \
  "cat /tmp/isaac_vslam_post_fix.log" \
  > "${OUTPUT_DIR}/isaac_vslam_post_fix.txt"

cat > "${OUTPUT_DIR}/summary.txt" <<EOF
Isaac ROS VSLAM evidence bundle

Generated: ${STAMP}
Container: ${CONTAINER_NAME}

Key files:
- realsense_device.txt
- visual_slam_status_once.txt
- visual_slam_odometry_once.txt
- visual_slam_odometry_hz.txt
- camera_infra1_hz.txt
- camera_imu_hz.txt
- isaac_vslam_launch_tail.txt
- isaac_vslam_post_fix.txt
EOF

echo "Saved VSLAM evidence to ${OUTPUT_DIR}"
