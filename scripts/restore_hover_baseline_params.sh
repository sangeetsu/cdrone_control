#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PROFILE_PATH="${REPO_ROOT}/ros2/src/drone_bringup/config/hover_baseline_profile.yaml"

exec python3 "${SCRIPT_DIR}/apply_px4_speed_profile.py" "${PROFILE_PATH}" "$@"
