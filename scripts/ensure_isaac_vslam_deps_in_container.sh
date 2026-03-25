#!/usr/bin/env bash

set -euo pipefail

CONTAINER_NAME="${ISAAC_ROS_CONTAINER_NAME:-isaac_ros_dev-aarch64-container}"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is not running." >&2
  echo "Start it with ./scripts/start_isaac_realsense_container.sh first." >&2
  exit 1
fi

required_packages=(
  ros-humble-isaac-ros-visual-slam
  ros-humble-isaac-ros-examples
  ros-humble-isaac-ros-realsense
  curl
  jq
  tar
)

missing_packages=()
for package_name in "${required_packages[@]}"; do
  if ! docker exec "${CONTAINER_NAME}" dpkg-query -W -f='${Status}\n' "${package_name}" 2>/dev/null \
    | grep -Fq 'install ok installed'; then
    missing_packages+=("${package_name}")
  fi
done

if [[ ${#missing_packages[@]} -eq 0 ]]; then
  echo "Isaac ROS VSLAM packages are already installed in ${CONTAINER_NAME}."
  exit 0
fi

echo "Installing missing packages in ${CONTAINER_NAME}:"
printf '  %s\n' "${missing_packages[@]}"

docker exec "${CONTAINER_NAME}" /bin/bash -lc \
  "apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y ${missing_packages[*]}"

echo "Isaac ROS VSLAM dependencies are ready in ${CONTAINER_NAME}."
