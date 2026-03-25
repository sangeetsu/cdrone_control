#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-${REPO_ROOT}/external/isaac_ros_release32}"
ISAAC_SRC_DIR="${ISAAC_ROS_WS}/src"
ISAAC_COMMON_DIR="${ISAAC_SRC_DIR}/isaac_ros_common"
ISAAC_COMMON_SCRIPTS_DIR="${ISAAC_COMMON_DIR}/scripts"

echo "Using ISAAC_ROS_WS=${ISAAC_ROS_WS}"

mkdir -p "${ISAAC_SRC_DIR}"

if [[ ! -d "${ISAAC_COMMON_DIR}/.git" ]]; then
  git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git "${ISAAC_COMMON_DIR}"
else
  git -C "${ISAAC_COMMON_DIR}" fetch origin release-3.2 --tags
  git -C "${ISAAC_COMMON_DIR}" checkout release-3.2
  git -C "${ISAAC_COMMON_DIR}" pull --ff-only origin release-3.2
fi

cat > "${ISAAC_COMMON_SCRIPTS_DIR}/.isaac_ros_common-config" <<'EOF'
CONFIG_IMAGE_KEY=ros2_humble.realsense
EOF

echo
echo "Isaac ROS common is ready at:"
echo "  ${ISAAC_COMMON_DIR}"
echo
echo "Configured image key:"
echo "  ros2_humble.realsense"
echo
echo "Next:"
echo "  ${REPO_ROOT}/scripts/run_isaac_realsense_dev.sh"
