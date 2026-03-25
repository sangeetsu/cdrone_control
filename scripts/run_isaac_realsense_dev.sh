#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ISAAC_ROS_WS="${ISAAC_ROS_WS:-${REPO_ROOT}/external/isaac_ros_release32}"
ISAAC_COMMON_DIR="${ISAAC_ROS_WS}/src/isaac_ros_common"

# Isaac's launcher uses `tput` for color output and exits under `set -e`
# if TERM is unset in a non-interactive shell.
export TERM="${TERM:-xterm-256color}"

if [[ ! -x "${ISAAC_COMMON_DIR}/scripts/run_dev.sh" ]]; then
  echo "Isaac ROS common is missing. Run ${REPO_ROOT}/scripts/setup_isaac_ros_release32.sh first." >&2
  exit 1
fi

if [[ ! -t 0 || ! -t 1 ]]; then
  echo "This wrapper expects an interactive TTY." >&2
  echo "For headless sessions, use ${REPO_ROOT}/scripts/start_isaac_realsense_container.sh instead." >&2
  exit 1
fi

cd "${ISAAC_COMMON_DIR}"
./scripts/run_dev.sh -d "${ISAAC_ROS_WS}" "$@"
