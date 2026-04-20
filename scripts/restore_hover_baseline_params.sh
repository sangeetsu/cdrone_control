#!/usr/bin/env bash
set -euo pipefail

PX4_SHELL="${PX4_SHELL:-/home/jetson/ARK-OS/platform/common/scripts/px4_shell_command.py}"
PX4_PORT="${PX4_PORT:-udp:127.0.0.1:14540}"

PARAMS=(
  "MIS_TAKEOFF_ALT=1.0"
  "MPC_XY_VEL_MAX=12.0"
  "MPC_VEL_MANUAL=10.0"
  "MPC_JERK_MAX=8.0"
  "MPC_XY_CRUISE=5.0"
  "MPC_JERK_AUTO=4.0"
  "MPC_Z_V_AUTO_UP=3.0"
  "MPC_ACC_HOR=3.0"
  "MPC_Z_VEL_MAX_DN=1.5"
  "MPC_Z_V_AUTO_DN=1.5"
  "MPC_TKO_SPEED=1.5"
  "MPC_LAND_SPEED=0.7"
  "CAL_ACC0_XOFF=-0.084256619215011597"
  "CAL_ACC0_YOFF=-0.051904574036598206"
  "CAL_ACC0_ZOFF=0.077345125377178192"
)

build_command() {
  local line name value

  for line in "${PARAMS[@]}"; do
    name="${line%%=*}"
    value="${line#*=}"
    printf 'param set %s %s\n' "$name" "$value"
  done

  printf 'param save\n'

  for line in "${PARAMS[@]}"; do
    name="${line%%=*}"
    printf 'param show %s\n' "$name"
  done
}

COMMANDS="$(build_command)"
python3 "${PX4_SHELL}" --port "${PX4_PORT}" "${COMMANDS}"
