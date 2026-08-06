#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DRONE_ID="cdrone4"
DRONE_NAMESPACE="/cdrone/cdrone4"
MAVROS_NAMESPACE="/cdrone/cdrone4/mavros"
MOCAP_NAMESPACE="/cdrone/cdrone4/vrpn_mocap"
RIGID_BODY_NAME="RigidBody4"
SOURCE_POSE_TOPIC="/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose"
FRAME_RPY_RAD="[0.0, 0.0, 3.141592653589793]"
OPTITRACK_SERVER="192.168.0.217"
AIR_TIME_S="3"
START_FLIGHT=0
SKIP_ROUTER_START=0
USE_BAREBONES=1
LAUNCH_PID=""
ROS_LOG_DIR="${REPO_ROOT}/flight_logs/quick_3s_hover/ros_log_$(date -u +%Y%m%dT%H%M%SZ)"
READY_ERROR=""

usage() {
  cat <<'EOF'
Usage:
  scripts/quick_cdrone4_3s_hover.sh [--start] [--skip-router-start]

Quick cdrone4/RigidBody4 hop using current PX4 parameters:
  - by default launches a barebones MAVROS + manual-control path for indoor bench use
  - optionally launches the full external-pose bridge with --pose-bridge
  - writes no PX4 parameters
  - applies no speed profile
  - with --start: set ALTCTL/STABILIZED, arm, briefly apply throttle, wait 3 seconds, then disarm/land

Default behavior is prepare-only. It launches the stack, checks readiness, and
prints the exact command to start the hop. Add --start only when the vehicle area
is clear and the pilot is ready to take over.

Options:
  --start              After readiness, command the barebones arm/spin sequence.
  --pose-bridge       Use the full external-pose bridge launch instead of the barebones path.
  --skip-router-start  Do not try to start mavlink-routerd if it is missing.
  -h, --help           Show this help.
EOF
}

log() {
  printf '[quick_3s_hover] %s\n' "$*"
}

warn() {
  printf '[quick_3s_hover] WARN: %s\n' "$*" >&2
}

fail() {
  printf '[quick_3s_hover] ERROR: %s\n' "$*" >&2
  exit 1
}

ready_fail() {
  READY_ERROR="$*"
  warn "$*"
  return 1
}

source_ros_env() {
  mkdir -p "${ROS_LOG_DIR}"
  export ROS_LOG_DIR

  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "${HOME}/.bashrc" ]]; then
    # shellcheck source=/dev/null
    source "${HOME}/.bashrc"
  fi
  if [[ -f "${REPO_ROOT}/ros2/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${REPO_ROOT}/ros2/install/setup.bash"
  fi
  set -u

  command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available after sourcing ROS/workspace setup."
}

ensure_router() {
  if pgrep -x mavlink-routerd >/dev/null 2>&1; then
    log "mavlink-routerd is already running."
    return
  fi

  if [[ "${SKIP_ROUTER_START}" -eq 1 ]]; then
    warn "mavlink-routerd is not running and --skip-router-start was set."
    return
  fi

  local start_script="/home/jetson/.local/bin/start_mavlink_router.sh"
  if [[ ! -x "${start_script}" ]]; then
    warn "Router start script is not executable: ${start_script}"
    return
  fi

  log "Starting mavlink-routerd with ${start_script}"
  nohup "${start_script}" >/tmp/quick_cdrone4_3s_hover_mavlink_router.log 2>&1 &
  for _ in $(seq 1 20); do
    if pgrep -x mavlink-routerd >/dev/null 2>&1; then
      log "mavlink-routerd started."
      return
    fi
    sleep 1
  done

  warn "mavlink-routerd did not appear within 20 seconds."
  warn "Router log: /tmp/quick_cdrone4_3s_hover_mavlink_router.log"
}

field_once() {
  local topic="$1"
  local field="$2"

  timeout 8s ros2 topic echo --no-daemon --spin-time 2 --once --field "${field}" "${topic}" 2>/dev/null \
    | sed -n '/^---$/d;/^WARNING:/d;/./{s/^[[:space:]]*//;p;q}'
}

wait_for_service() {
  local service="$1"
  for _ in $(seq 1 30); do
    if [[ -n "${LAUNCH_PID}" ]] && ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      fail "Launch process exited before service appeared: ${service}"
    fi
    if ros2 service list --no-daemon --spin-time 1 2>/dev/null | grep -Fxq "${service}"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

call_trigger() {
  local service="$1"
  local output

  output="$(timeout 10s ros2 service call "${service}" std_srvs/srv/Trigger "{}" 2>&1 || true)"
  printf '%s\n' "${output}"
  grep -Eiq 'success[:=][[:space:]]*(true|True)|success=True' <<<"${output}"
}

set_mode() {
  local mode="$1"
  local output

  output="$(timeout 10s ros2 service call "${MAVROS_NAMESPACE}/set_mode" mavros_msgs/srv/SetMode "{custom_mode: '${mode}'}" 2>&1 || true)"
  printf '%s\n' "${output}"
  grep -Eiq 'mode_sent[:=][[:space:]]*(true|True)|mode_sent=True' <<<"${output}"
}

arm_vehicle() {
  local output

  output="$(timeout 10s ros2 service call "${MAVROS_NAMESPACE}/cmd/arming" mavros_msgs/srv/CommandBool "{value: true}" 2>&1 || true)"
  printf '%s\n' "${output}"
  grep -Eiq 'success[:=][[:space:]]*(true|True)|success=True' <<<"${output}"
}

check_ready() {
  local connected
  local armed
  local companion_state

  wait_for_service "${MAVROS_NAMESPACE}/set_mode" || ready_fail "Missing service: ${MAVROS_NAMESPACE}/set_mode"
  wait_for_service "${MAVROS_NAMESPACE}/cmd/arming" || ready_fail "Missing service: ${MAVROS_NAMESPACE}/cmd/arming"

  connected="$(field_once "${MAVROS_NAMESPACE}/state" connected | tr '[:upper:]' '[:lower:]')"
  [[ "${connected}" == "true" ]] || ready_fail "MAVROS is not connected. connected=${connected:-missing}"

  armed="$(field_once "${MAVROS_NAMESPACE}/state" armed | tr '[:upper:]' '[:lower:]')"
  [[ "${armed}" == "false" ]] || ready_fail "Vehicle is already armed; refusing to start."

  if [[ "${USE_BAREBONES}" -eq 1 ]]; then
    timeout 8s ros2 topic echo --no-daemon --spin-time 2 --once "${MAVROS_NAMESPACE}/local_position/pose" >/dev/null 2>&1 \
      || ready_fail "No MAVROS local position sample."
    log "Ready: MAVROS connected, unarmed, barebones manual-control path active."
    return 0
  fi

  timeout 8s ros2 topic echo --no-daemon --spin-time 2 --once "${SOURCE_POSE_TOPIC}" >/dev/null 2>&1 \
    || ready_fail "No RigidBody4 source pose sample from ${SOURCE_POSE_TOPIC}"
  timeout 8s ros2 topic echo --no-daemon --spin-time 2 --once "${MAVROS_NAMESPACE}/local_position/pose" >/dev/null 2>&1 \
    || ready_fail "No MAVROS local position sample."
  timeout 8s ros2 topic echo --no-daemon --spin-time 2 --once "${MAVROS_NAMESPACE}/vision_pose/pose" >/dev/null 2>&1 \
    || ready_fail "No MAVROS vision pose sample."

  companion_state="$(field_once "${MAVROS_NAMESPACE}/companion_process/status" state)"
  [[ "${companion_state}" == "4" ]] || ready_fail "External pose companion status is not ACTIVE. state=${companion_state:-missing}"

  log "Ready: MAVROS connected, unarmed, pose streams live."
}

print_inspection_commands() {
  log "Inspection commands while this launch stays alive:"
  printf '  ros2 node list\n'
  printf '  ros2 topic list | grep -E %q\n' 'RigidBody4|vrpn|external_pose|vision_pose|local_position|mavros/state'
  printf '  ros2 topic hz %q\n' "${SOURCE_POSE_TOPIC}"
  printf '  ros2 topic echo --once %q\n' "${SOURCE_POSE_TOPIC}"
  printf '  ros2 topic hz %q\n' "${MAVROS_NAMESPACE}/vision_pose/pose"
  printf '  ros2 topic echo --once %q\n' "${MAVROS_NAMESPACE}/state"
}

publish_manual_control() {
  local throttle_z="$1"
  local topic="${MAVROS_NAMESPACE}/manual_control/send"
  timeout 10s ros2 topic pub --once "${topic}" mavros_msgs/msg/ManualControl "{x: 0.0, y: 0.0, z: ${throttle_z}, r: 0.0, buttons: 0}" >/dev/null 2>&1 || true
}

disarm_vehicle() {
  local output

  output="$(timeout 10s ros2 service call "${MAVROS_NAMESPACE}/cmd/arming" mavros_msgs/srv/CommandBool "{value: false}" 2>&1 || true)"
  printf '%s\n' "${output}"
}

request_land() {
  warn "Requesting AUTO.LAND."
  set_mode "AUTO.LAND" >/tmp/quick_cdrone4_3s_hover_land.log 2>&1 || true
}

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [[ "${code}" -ne 0 && "${START_FLIGHT}" -eq 1 ]]; then
    request_land
  fi
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    kill -INT "${LAUNCH_PID}" >/dev/null 2>&1 || true
    sleep 1
    kill -TERM "${LAUNCH_PID}" >/dev/null 2>&1 || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  exit "${code}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --start)
      START_FLIGHT=1
      shift
      ;;
    --pose-bridge)
      USE_BAREBONES=0
      shift
      ;;
    --skip-router-start)
      SKIP_ROUTER_START=1
      shift
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

trap cleanup EXIT INT TERM

source_ros_env
ensure_router

if [[ "${USE_BAREBONES}" -eq 1 ]]; then
  log "Launching barebones MAVROS stack for ${DRONE_ID}; no PX4 parameter writes and no pose bridge."
  ros2 launch drone_bringup drone.launch.py \
    mavros_namespace:="${MAVROS_NAMESPACE}" &
else
  log "Launching MAVROS + external pose for ${DRONE_ID}/${RIGID_BODY_NAME}; no PX4 parameter writes."
  ros2 launch drone_bringup external_pose_px4_bridge.launch.py \
    drone_id:="${DRONE_ID}" \
    drone_namespace:="${DRONE_NAMESPACE}" \
    mavros_namespace:="${MAVROS_NAMESPACE}" \
    mocap_namespace:="${MOCAP_NAMESPACE}" \
    pose_source:=optitrack \
    optitrack_server:="${OPTITRACK_SERVER}" \
    rigid_body_name:="${RIGID_BODY_NAME}" \
    source_pose_topic:="${SOURCE_POSE_TOPIC}" \
    frame_rpy_rad:="${FRAME_RPY_RAD}" &
fi
LAUNCH_PID="$!"

if ! check_ready; then
  if [[ "${START_FLIGHT}" -eq 0 ]]; then
    warn "Prepare check failed: ${READY_ERROR:-unknown readiness failure}"
    warn "Keeping the launch alive for debugging. Press Ctrl-C here when done."
    print_inspection_commands
    wait "${LAUNCH_PID}"
    exit 1
  fi
  fail "Readiness failed; refusing to start flight: ${READY_ERROR:-unknown readiness failure}"
fi

if [[ "${START_FLIGHT}" -eq 0 ]]; then
  log "Prepared only. To command the 3-second hop, rerun:"
  printf '  %q --start\n' "${SCRIPT_DIR}/quick_cdrone4_3s_hover.sh"
  log "Land command if needed:"
  printf '  ros2 service call %q mavros_msgs/srv/SetMode "{custom_mode: '\''AUTO.LAND'\''}"\n' "${MAVROS_NAMESPACE}/set_mode"
  wait "${LAUNCH_PID}"
  exit 0
fi

if [[ "${USE_BAREBONES}" -eq 1 ]]; then
  log "Setting ALTCTL for manual-control bench mode."
  set_mode "ALTCTL" || warn "ALTCTL mode request was not accepted; trying STABILIZED."
  set_mode "STABILIZED" >/tmp/quick_cdrone4_3s_hover_mode.log 2>&1 || true

  sleep 1
  log "Publishing zero throttle to prepare for arming."
  publish_manual_control 0

  sleep 1
  log "Arming."
  arm_vehicle || fail "Arming request was not accepted."

  sleep 1
  log "Applying lightweight throttle to spin motors briefly."
  publish_manual_control 600

  log "Air-time hold: ${AIR_TIME_S}s, then disarm."
  sleep "${AIR_TIME_S}"

  log "Stopping throttle and disarming."
  publish_manual_control 0
  disarm_vehicle || true
  set_mode "AUTO.LAND" >/tmp/quick_cdrone4_3s_hover_land.log 2>&1 || true
  log "Barebones sequence complete."
  exit 0
fi

log "Setting AUTO.TAKEOFF."
set_mode "AUTO.TAKEOFF" || fail "AUTO.TAKEOFF mode request was not accepted."

sleep 1

log "Arming."
arm_vehicle || fail "Arming request was not accepted."

log "Air-time hold: ${AIR_TIME_S}s, then AUTO.LAND."
sleep "${AIR_TIME_S}"

log "Setting AUTO.LAND."
set_mode "AUTO.LAND" || fail "AUTO.LAND mode request was not accepted."

log "Landing requested. Watch QGC/RC until landed and disarmed."
