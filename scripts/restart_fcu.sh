#!/usr/bin/env bash
# restart_fcu.sh
# Reboots the PX4 flight controller (FCU) via MAVROS using
# MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (command=246, param1=1.0).

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2/install/setup.bash"

MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-mavros}"
MAVROS_NAMESPACE="${MAVROS_NAMESPACE#/}"
MAVROS_SERVICE="/${MAVROS_NAMESPACE}/cmd/command"
MAVROS_STATE_TOPIC="/${MAVROS_NAMESPACE}/state"

WAIT_SECONDS="${WAIT_SECONDS:-20}"
AUTO_START_MAVROS=1
DRY_RUN=0

STARTED_MAVROS=0
MAVROS_LAUNCH_PID=""
MAVROS_LAUNCH_LOG=""

usage() {
  cat <<EOF
Usage: $(basename "$0") [--dry-run] [--no-autostart] [--wait-seconds N]

Options:
  --dry-run         Check prerequisites and MAVROS/FCU connectivity, but do not reboot.
  --no-autostart    Do not auto-launch MAVROS if /${MAVROS_NAMESPACE}/cmd/command is missing.
  --wait-seconds N  Timeout used while waiting for MAVROS/service connection. Default: ${WAIT_SECONDS}
  -h, --help        Show this help text.

Environment:
  MAVROS_NAMESPACE  MAVROS namespace to use. Default: mavros
  WAIT_SECONDS      Default wait timeout in seconds.

Notes:
  This script sends the reboot through MAVROS, so MAVROS must be running and
  connected to the FCU. With the current repo config, mavlink-router must also
  be running because MAVROS connects to udp://:14540@127.0.0.1:14550.
EOF
}

log() {
  printf '[restart_fcu] %s\n' "$*"
}

fail() {
  printf '[restart_fcu] ERROR: %s\n' "$*" >&2
  if [[ -n "${MAVROS_LAUNCH_LOG}" && -f "${MAVROS_LAUNCH_LOG}" ]]; then
    printf '[restart_fcu] Last MAVROS log lines (%s):\n' "${MAVROS_LAUNCH_LOG}" >&2
    tail -n 40 "${MAVROS_LAUNCH_LOG}" >&2 || true
  fi
  exit 1
}

cleanup() {
  if [[ "${STARTED_MAVROS}" -eq 1 && -n "${MAVROS_LAUNCH_PID}" ]]; then
    log "Stopping temporary MAVROS launch (pid ${MAVROS_LAUNCH_PID})."
    kill "${MAVROS_LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${MAVROS_LAUNCH_PID}" >/dev/null 2>&1 || true
  fi
}

have_command() {
  command -v "$1" >/dev/null 2>&1
}

require_file() {
  local path="$1"
  [[ -f "${path}" ]] || fail "Required file not found: ${path}"
}

source_if_exists() {
  local path="$1"
  if [[ -f "${path}" ]]; then
    local restore_nounset=0
    if [[ "$-" == *u* ]]; then
      restore_nounset=1
      set +u
    fi
    # shellcheck source=/dev/null
    source "${path}"
    if [[ "${restore_nounset}" -eq 1 ]]; then
      set -u
    fi
  fi
}

service_exists() {
  local services=""
  services="$(ros2 service list 2>/dev/null || true)"
  grep -Fxq -- "${MAVROS_SERVICE}" <<<"${services}"
}

wait_for_service() {
  local deadline=$((SECONDS + WAIT_SECONDS))
  while (( SECONDS < deadline )); do
    if service_exists; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_fcu_connection() {
  local deadline=$((SECONDS + WAIT_SECONDS))
  local connected=""

  while (( SECONDS < deadline )); do
    connected="$(timeout 2 ros2 topic echo --once --field connected "${MAVROS_STATE_TOPIC}" 2>/dev/null || true)"
    if printf '%s\n' "${connected}" | tr '[:upper:]' '[:lower:]' | grep -Fq 'true'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

start_mavros_if_needed() {
  if service_exists; then
    return 0
  fi

  if [[ "${AUTO_START_MAVROS}" -ne 1 ]]; then
    fail "MAVROS service ${MAVROS_SERVICE} is not available. Start MAVROS first with: ros2 launch drone_bringup drone.launch.py"
  fi

  if ! ros2 pkg prefix drone_bringup >/dev/null 2>&1; then
    fail "Package drone_bringup is not discoverable. Build/source the workspace before auto-starting MAVROS."
  fi

  MAVROS_LAUNCH_LOG="$(mktemp -t restart_fcu_mavros.XXXXXX.log)"
  log "MAVROS service ${MAVROS_SERVICE} is missing; starting MAVROS via drone.launch.py."
  ros2 launch --noninteractive drone_bringup drone.launch.py "mavros_namespace:=${MAVROS_NAMESPACE}" >"${MAVROS_LAUNCH_LOG}" 2>&1 &
  MAVROS_LAUNCH_PID=$!
  STARTED_MAVROS=1

  if ! wait_for_service; then
    fail "Timed out waiting for ${MAVROS_SERVICE} after starting MAVROS."
  fi
}

while (($# > 0)); do
  case "$1" in
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    --no-autostart)
      AUTO_START_MAVROS=0
      shift
      ;;
    --wait-seconds)
      [[ $# -ge 2 ]] || fail "Missing value after --wait-seconds"
      WAIT_SECONDS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      fail "Unknown argument: $1"
      ;;
  esac
done

trap cleanup EXIT

have_command ros2 || fail "ros2 CLI is not installed or not on PATH."
have_command timeout || fail "timeout is required but not installed."

require_file "${ROS_SETUP}"
source_if_exists "${ROS_SETUP}"
source_if_exists "${WORKSPACE_SETUP}"

if ! pgrep -f "mavlink-routerd|start_mavlink_router.sh" >/dev/null 2>&1; then
  fail "mavlink-router is not running. Start /home/jetson/.local/bin/start_mavlink_router.sh first."
fi

start_mavros_if_needed

if ! wait_for_fcu_connection; then
  fail "MAVROS is up, but ${MAVROS_STATE_TOPIC} never reported connected=true. Check FCU power, USB, and mavlink-router."
fi

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "Dry run successful. MAVROS is reachable and connected to the FCU."
  exit 0
fi

log "Sending reboot command to FCU via ${MAVROS_SERVICE} ..."

SERVICE_OUTPUT="$(timeout 10 ros2 service call "${MAVROS_SERVICE}" mavros_msgs/srv/CommandLong \
  "{broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}" 2>&1)" \
  || fail "Reboot service call failed."

printf '%s\n' "${SERVICE_OUTPUT}"

if ! printf '%s\n' "${SERVICE_OUTPUT}" | grep -Eq 'success[:=][[:space:]]*(true|True)'; then
  fail "MAVROS responded, but the FCU reboot command was not acknowledged as successful."
fi

log "Reboot command acknowledged. FCU should restart momentarily."
