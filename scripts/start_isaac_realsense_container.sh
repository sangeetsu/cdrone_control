#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-${REPO_ROOT}/external/isaac_ros_release32}"
CONTAINER_NAME="${ISAAC_ROS_CONTAINER_NAME:-isaac_ros_dev-aarch64-container}"
IMAGE_NAME="${ISAAC_ROS_IMAGE_NAME:-isaac_ros_dev-aarch64}"

if [[ ! -d "${ISAAC_ROS_WS}" ]]; then
  echo "Isaac ROS workspace is missing at ${ISAAC_ROS_WS}." >&2
  echo "Run ${REPO_ROOT}/scripts/setup_isaac_ros_release32.sh first." >&2
  exit 1
fi

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "Docker image ${IMAGE_NAME} is missing." >&2
  echo "Build it once with ${REPO_ROOT}/scripts/run_isaac_realsense_dev.sh from an interactive terminal." >&2
  exit 1
fi

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
  echo "${CONTAINER_NAME} is already running."
  exit 0
fi

# Stay on the classic Jetson NVIDIA runtime path. The release-3.2 aarch64
# launcher also tries CDI-style visible devices, but this machine does not
# currently have resolvable CDI names for `nvidia.com/gpu=all`.
docker run -d --rm \
  --privileged \
  --network host \
  --ipc=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "${HOME}/.Xauthority:/home/admin/.Xauthority:rw" \
  -e DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_DOMAIN_ID \
  -e USER \
  -e ISAAC_ROS_WS=/workspaces/isaac_ros-dev \
  -e HOST_USER_UID="$(id -u)" \
  -e HOST_USER_GID="$(id -g)" \
  -v /usr/bin/tegrastats:/usr/bin/tegrastats \
  -v /tmp/:/tmp/ \
  -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
  -v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api \
  --pid=host \
  -v /usr/share/vpi3:/usr/share/vpi3 \
  -v /dev/input:/dev/input \
  -v /run/jtop.sock:/run/jtop.sock:ro \
  -v "${ISAAC_ROS_WS}:/workspaces/isaac_ros-dev" \
  -v /etc/localtime:/etc/localtime:ro \
  --name "${CONTAINER_NAME}" \
  --runtime nvidia \
  --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
  --workdir /workspaces/isaac_ros-dev \
  "${IMAGE_NAME}" \
  /bin/bash -lc 'sleep infinity'

echo "Started ${CONTAINER_NAME}."
echo "Stop it with: docker stop ${CONTAINER_NAME}"
