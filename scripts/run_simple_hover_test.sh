#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

DRONE_ID="${SIMPLE_HOVER_DRONE_ID:-cdrone4}"
DRONE_NAMESPACE="${SIMPLE_HOVER_DRONE_NAMESPACE:-}"
MAVROS_NAMESPACE="${SIMPLE_HOVER_MAVROS_NAMESPACE:-}"
MOCAP_NAMESPACE="${SIMPLE_HOVER_MOCAP_NAMESPACE:-}"
RIGID_BODY_NAME="${SIMPLE_HOVER_RIGID_BODY_NAME:-RigidBody4}"
SOURCE_POSE_TOPIC="${SIMPLE_HOVER_SOURCE_POSE_TOPIC:-}"
FRAME_RPY_RAD="${SIMPLE_HOVER_FRAME_RPY_RAD:-[0.0, 0.0, 3.141592653589793]}"
OPTITRACK_SERVER="${SIMPLE_HOVER_OPTITRACK_SERVER:-192.168.0.217}"
TAKEOFF_ALTITUDE_M="${SIMPLE_HOVER_TAKEOFF_ALTITUDE_M:-1.0}"
HOVER_DURATION_S="${SIMPLE_HOVER_HOVER_DURATION_S:-10.0}"
TAKEOFF_RATE_M_S="${SIMPLE_HOVER_TAKEOFF_RATE_M_S:-0.5}"
MAX_HORIZONTAL_EXCURSION_M="${SIMPLE_HOVER_MAX_HORIZONTAL_EXCURSION_M:-0.75}"
CAPTURE_DURATION_S="${SIMPLE_HOVER_CAPTURE_DURATION_S:-240}"
READY_TIMEOUT_S="${SIMPLE_HOVER_READY_TIMEOUT_S:-90}"
READY_RETRY_DELAY_S="${SIMPLE_HOVER_READY_RETRY_DELAY_S:-2}"
MAVLINK_ROUTER_START_SCRIPT="${SIMPLE_HOVER_MAVLINK_ROUTER_START_SCRIPT:-/home/jetson/.local/bin/start_mavlink_router.sh}"
MAVLINK_ROUTER_START_TIMEOUT_S="${SIMPLE_HOVER_MAVLINK_ROUTER_START_TIMEOUT_S:-20}"
RUN_ID="${SIMPLE_HOVER_RUN_ID:-}"
OUTPUT_DIR="${SIMPLE_HOVER_OUTPUT_DIR:-}"
RESTORE_BASELINE=1
AUTO_START_ROUTER=1
DRY_RUN=0
STOPPED_JOBS=0

declare -a JOB_PIDS=()
LAUNCH_PID=""
CAPTURE_PID=""
LATEST_DEMO_STATE=""

usage() {
  cat <<'EOF'
Usage:
  scripts/run_simple_hover_test.sh [options]

Launches the cdrone4/RigidBody4 simple-hover stack, starts debug capture, waits
for readiness, and prints the explicit start/abort commands. It does not call
the start service automatically.

Options:
  --drone-id ID                 Drone id. Default: cdrone4
  --drone-namespace NS          Drone namespace. Default: /cdrone/<drone-id>
  --mavros-namespace NS         MAVROS namespace. Default: /cdrone/<drone-id>/mavros
  --mocap-namespace NS          Mocap namespace. Default: /cdrone/<drone-id>/vrpn_mocap
  --rigid-body-name NAME        Mocap rigid body. Default: RigidBody4
  --source-pose-topic TOPIC     Source pose topic. Default: <mocap-ns>/<rigid-body>/pose
  --frame-rpy-rad VECTOR        External-pose frame rotation. Default: yaw pi
  --optitrack-server IP         OptiTrack server. Default: 192.168.0.217
  --takeoff-altitude-m M        Takeoff altitude delta. Default: 1.0
  --hover-duration-s SEC        Hover dwell. Default: 10.0
  --takeoff-rate-m-s MPS        Takeoff rate. Default: 0.5
  --max-horizontal-excursion-m M  Abort threshold. Default: 0.75
  --capture-duration SEC        Debug capture max duration. Default: 240
  --ready-timeout SEC           Total readiness wait. Default: 90
  --ready-retry-delay SEC       Delay between readiness attempts. Default: 2
  --mavlink-router-start PATH   Router start script. Default: /home/jetson/.local/bin/start_mavlink_router.sh
  --mavlink-router-timeout SEC  Router startup wait. Default: 20
  --run-id ID                   Run id. Default: simple_hover_<drone>_<stamp>
  --output-dir PATH             Output directory. Default: flight_logs/simple_hover/<run-id>
  --skip-baseline-restore       Do not run restore_hover_baseline_params.sh.
  --no-router-autostart         Do not start mavlink-routerd automatically.
  --dry-run                     Print commands without changing anything.
  -h, --help                    Show this help.
EOF
}

log() {
  printf '[simple_hover_run] %s\n' "$*"
}

warn() {
  printf '[simple_hover_run] WARN: %s\n' "$*" >&2
}

fail() {
  printf '[simple_hover_run] ERROR: %s\n' "$*" >&2
  exit 1
}

normalize_ns() {
  local namespace="$1"
  namespace="${namespace:-}"
  namespace="${namespace%/}"
  if [[ -z "${namespace}" ]]; then
    printf '/'
    return
  fi
  if [[ "${namespace}" != /* ]]; then
    namespace="/${namespace}"
  fi
  printf '%s' "${namespace}"
}

join_topic() {
  local namespace="$1"
  local leaf="$2"
  namespace="$(normalize_ns "${namespace}")"
  leaf="/${leaf#/}"
  if [[ "${namespace}" == "/" ]]; then
    printf '%s' "${leaf}"
  else
    printf '%s%s' "${namespace}" "${leaf}"
  fi
}

validate_positive_number() {
  local name="$1"
  local value="$2"
  python3 - "$name" "$value" <<'PY'
import sys

name = sys.argv[1]
raw = sys.argv[2]
try:
    value = float(raw)
except ValueError as exc:
    raise SystemExit(f"{name} must be a positive number, got {raw!r}") from exc
if value <= 0.0:
    raise SystemExit(f"{name} must be greater than zero, got {raw!r}")
PY
}

validate_positive_integer() {
  local name="$1"
  local value="$2"
  if ! [[ "${value}" =~ ^[1-9][0-9]*$ ]]; then
    fail "${name} must be a positive integer number of seconds, got '${value}'."
  fi
}

is_mavlink_router_running() {
  pgrep -x "mavlink-routerd" >/dev/null 2>&1
}

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

source_ros_env() {
  local restore_nounset=0
  if [[ "$-" == *u* ]]; then
    restore_nounset=1
    set +u
  fi

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

  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available after sourcing ROS and workspace setup files."
}

ensure_mavlink_router() {
  local start_s
  local now_s

  if is_mavlink_router_running; then
    log "mavlink-routerd is already running."
    return 0
  fi

  if [[ "${AUTO_START_ROUTER}" -eq 0 ]]; then
    fail "mavlink-routerd is not running. Start ${MAVLINK_ROUTER_START_SCRIPT} or rerun without --no-router-autostart."
  fi

  if [[ ! -x "${MAVLINK_ROUTER_START_SCRIPT}" ]]; then
    fail "mavlink-routerd is not running and router start script is not executable: ${MAVLINK_ROUTER_START_SCRIPT}"
  fi

  log "mavlink-routerd is not running; starting ${MAVLINK_ROUTER_START_SCRIPT}"
  nohup "${MAVLINK_ROUTER_START_SCRIPT}" \
    > "${MAVLINK_ROUTER_LOG}" 2>&1 &

  start_s="$(date +%s)"
  while true; do
    if is_mavlink_router_running; then
      log "mavlink-routerd started."
      return 0
    fi

    now_s="$(date +%s)"
    if (( now_s - start_s >= MAVLINK_ROUTER_START_TIMEOUT_S )); then
      warn "mavlink-routerd did not start within ${MAVLINK_ROUTER_START_TIMEOUT_S}s."
      if [[ -f "${MAVLINK_ROUTER_LOG}" ]]; then
        tail -n 40 "${MAVLINK_ROUTER_LOG}" || true
      fi
      fail "Cannot continue without mavlink-routerd. Check FC USB, ARK serial path, and ${MAVLINK_ROUTER_LOG}."
    fi

    sleep 1
  done
}

restore_hover_baseline() {
  local baseline_status

  ensure_mavlink_router

  log "Restoring hover baseline PX4 parameters."
  set +e
  "${BASELINE_CMD[@]}" 2>&1 | tee "${OUTPUT_DIR}/baseline_restore.log"
  baseline_status="${PIPESTATUS[0]}"
  set -e

  if [[ "${baseline_status}" -eq 0 ]]; then
    return 0
  fi

  warn "Hover baseline restore failed."
  warn "This means PX4 did not answer the MAVLink parameter probe on udpout:127.0.0.1:14550."
  warn "Check that the FC is powered, the ARK USB serial endpoint is present, and mavlink-routerd is attached to it."
  warn "Router log: ${MAVLINK_ROUTER_LOG}"
  warn "Baseline log: ${OUTPUT_DIR}/baseline_restore.log"
  warn "If you intentionally want to skip parameter restore, rerun with --skip-baseline-restore."
  return "${baseline_status}"
}

field_value_from_output() {
  local field="$1"
  local output="$2"
  FIELD_OUTPUT="${output}" python3 - "$field" <<'PY'
import os
import sys

field = sys.argv[1]
for raw_line in os.environ.get("FIELD_OUTPUT", "").splitlines():
    line = raw_line.strip()
    if not line or line == "---" or line.startswith("WARNING:"):
        continue
    if ":" in line:
        key, value = line.split(":", 1)
        if key.strip() == field or len(line.split(":", 1)) == 2:
            line = value.strip()
    print(line.strip("'\""))
    break
PY
}

read_demo_state_quick() {
  local output
  output="$(
    timeout 4s \
      ros2 topic echo --no-daemon --spin-time 1 \
      --once --field data "${DEMO_STATE_TOPIC}" 2>/dev/null || true
  )"
  field_value_from_output "data" "${output}"
}

signal_process_tree() {
  local pid="$1"
  local signal_name="$2"
  local child

  [[ -n "${pid}" ]] || return

  while IFS= read -r child; do
    [[ -z "${child}" ]] && continue
    signal_process_tree "${child}" "${signal_name}"
  done < <(pgrep -P "${pid}" || true)

  kill "-${signal_name}" "${pid}" >/dev/null 2>&1 || true
}

stop_background_jobs() {
  if [[ "${STOPPED_JOBS}" -eq 1 ]]; then
    return
  fi
  STOPPED_JOBS=1

  for pid in "${JOB_PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      signal_process_tree "${pid}" "INT"
    fi
  done

  sleep 1

  for pid in "${JOB_PIDS[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      signal_process_tree "${pid}" "TERM"
    fi
  done

  for pid in "${JOB_PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

request_abort_if_active() {
  local state

  [[ -n "${LAUNCH_PID}" ]] || return
  if ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
    return
  fi

  state="$(read_demo_state_quick || true)"
  case "${state}" in
    ""|IDLE|COMPLETE|ABORT)
      return
      ;;
  esac

  warn "Interrupted while demo state=${state}; requesting abort before shutdown."
  timeout 8s ros2 service call "${DEMO_ABORT_SERVICE}" std_srvs/srv/Trigger "{}" \
    > "${OUTPUT_DIR}/interrupt_abort_service.log" 2>&1 || true
  sleep 3
}

handle_exit() {
  local exit_code=$?
  trap - EXIT INT TERM
  set +e
  if [[ "${DRY_RUN}" -eq 0 && "${exit_code}" -ne 0 ]]; then
    request_abort_if_active
  fi
  stop_background_jobs
  exit "${exit_code}"
}

wait_for_ready() {
  local start_s
  local now_s
  local attempt=1

  start_s="$(date +%s)"
  while true; do
    log "Readiness attempt ${attempt}; writing ${READY_LOG}"
    if "${READY_CMD[@]}" > "${READY_LOG}" 2>&1; then
      log "Readiness checks passed."
      tail -n 12 "${READY_LOG}" || true
      return 0
    fi

    now_s="$(date +%s)"
    if (( now_s - start_s >= READY_TIMEOUT_S )); then
      warn "Readiness checks did not pass within ${READY_TIMEOUT_S}s."
      tail -n 40 "${READY_LOG}" || true
      return 1
    fi

    tail -n 8 "${READY_LOG}" || true
    sleep "${READY_RETRY_DELAY_S}"
    attempt=$((attempt + 1))
  done
}

monitor_until_terminal_state() {
  local state
  local seen_active=0

  log "Monitoring ${DEMO_STATE_TOPIC} until COMPLETE or ABORT."
  while true; do
    if [[ -n "${LAUNCH_PID}" ]] && ! kill -0 "${LAUNCH_PID}" >/dev/null 2>&1; then
      warn "Launch process exited before a terminal demo state."
      return 1
    fi

    state="$(read_demo_state_quick || true)"
    if [[ -n "${state}" ]]; then
      if [[ "${state}" != "${LATEST_DEMO_STATE}" ]]; then
        log "Demo state: ${state}"
        LATEST_DEMO_STATE="${state}"
      fi
      if [[ "${state}" != "IDLE" ]]; then
        seen_active=1
      fi
      if [[ "${seen_active}" -eq 1 && "${state}" == "COMPLETE" ]]; then
        log "Hover sequence completed."
        return 0
      fi
      if [[ "${seen_active}" -eq 1 && "${state}" == "ABORT" ]]; then
        warn "Hover sequence ended in ABORT."
        return 2
      fi
    fi

    sleep 1
  done
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --drone-id)
      DRONE_ID="$2"
      shift 2
      ;;
    --drone-namespace)
      DRONE_NAMESPACE="$2"
      shift 2
      ;;
    --mavros-namespace)
      MAVROS_NAMESPACE="$2"
      shift 2
      ;;
    --mocap-namespace)
      MOCAP_NAMESPACE="$2"
      shift 2
      ;;
    --rigid-body-name)
      RIGID_BODY_NAME="$2"
      shift 2
      ;;
    --source-pose-topic)
      SOURCE_POSE_TOPIC="$2"
      shift 2
      ;;
    --frame-rpy-rad)
      FRAME_RPY_RAD="$2"
      shift 2
      ;;
    --optitrack-server)
      OPTITRACK_SERVER="$2"
      shift 2
      ;;
    --takeoff-altitude-m)
      TAKEOFF_ALTITUDE_M="$2"
      shift 2
      ;;
    --hover-duration-s)
      HOVER_DURATION_S="$2"
      shift 2
      ;;
    --takeoff-rate-m-s)
      TAKEOFF_RATE_M_S="$2"
      shift 2
      ;;
    --max-horizontal-excursion-m)
      MAX_HORIZONTAL_EXCURSION_M="$2"
      shift 2
      ;;
    --capture-duration)
      CAPTURE_DURATION_S="$2"
      shift 2
      ;;
    --ready-timeout)
      READY_TIMEOUT_S="$2"
      shift 2
      ;;
    --ready-retry-delay)
      READY_RETRY_DELAY_S="$2"
      shift 2
      ;;
    --mavlink-router-start)
      MAVLINK_ROUTER_START_SCRIPT="$2"
      shift 2
      ;;
    --mavlink-router-timeout)
      MAVLINK_ROUTER_START_TIMEOUT_S="$2"
      shift 2
      ;;
    --run-id)
      RUN_ID="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-baseline-restore)
      RESTORE_BASELINE=0
      shift
      ;;
    --no-router-autostart)
      AUTO_START_ROUTER=0
      shift
      ;;
    --dry-run)
      DRY_RUN=1
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

validate_positive_number "--takeoff-altitude-m" "${TAKEOFF_ALTITUDE_M}"
validate_positive_number "--hover-duration-s" "${HOVER_DURATION_S}"
validate_positive_number "--takeoff-rate-m-s" "${TAKEOFF_RATE_M_S}"
validate_positive_number "--max-horizontal-excursion-m" "${MAX_HORIZONTAL_EXCURSION_M}"
validate_positive_integer "--capture-duration" "${CAPTURE_DURATION_S}"
validate_positive_integer "--ready-timeout" "${READY_TIMEOUT_S}"
validate_positive_number "--ready-retry-delay" "${READY_RETRY_DELAY_S}"
validate_positive_integer "--mavlink-router-timeout" "${MAVLINK_ROUTER_START_TIMEOUT_S}"

DRONE_ID="${DRONE_ID#/}"
DRONE_ID="${DRONE_ID%/}"
[[ -n "${DRONE_ID}" ]] || fail "--drone-id must not be empty."

DRONE_NAMESPACE="$(normalize_ns "${DRONE_NAMESPACE:-/cdrone/${DRONE_ID}}")"
MAVROS_NAMESPACE="$(normalize_ns "${MAVROS_NAMESPACE:-/cdrone/${DRONE_ID}/mavros}")"
MOCAP_NAMESPACE="$(normalize_ns "${MOCAP_NAMESPACE:-/cdrone/${DRONE_ID}/vrpn_mocap}")"
SOURCE_POSE_TOPIC="${SOURCE_POSE_TOPIC:-$(join_topic "${MOCAP_NAMESPACE}" "${RIGID_BODY_NAME}/pose")}"
RUN_ID="${RUN_ID:-simple_hover_${DRONE_ID}_${STAMP}}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/flight_logs/simple_hover/${RUN_ID}}"
CAPTURE_DIR="${OUTPUT_DIR}/capture"
LAUNCH_LOG="${OUTPUT_DIR}/position_hover_launch.log"
CAPTURE_LOG="${OUTPUT_DIR}/capture_position_hover_debug.log"
READY_LOG="${OUTPUT_DIR}/simple_hover_ready.log"
MAVLINK_ROUTER_LOG="${OUTPUT_DIR}/mavlink_router_start.log"
DEMO_STATE_TOPIC="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_state")"
DEMO_ABORT_SERVICE="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_abort")"
EXTERNAL_POSE_TOPIC="$(join_topic "${DRONE_NAMESPACE}" "external_pose/input_pose")"

BASELINE_CMD=("${SCRIPT_DIR}/restore_hover_baseline_params.sh")
LAUNCH_CMD=(
  ros2 launch drone_bringup position_hover_demo.launch.py
  "drone_id:=${DRONE_ID}"
  "drone_namespace:=${DRONE_NAMESPACE}"
  "mavros_namespace:=${MAVROS_NAMESPACE}"
  "mocap_namespace:=${MOCAP_NAMESPACE}"
  "pose_source:=optitrack"
  "optitrack_server:=${OPTITRACK_SERVER}"
  "rigid_body_name:=${RIGID_BODY_NAME}"
  "source_pose_topic:=${SOURCE_POSE_TOPIC}"
  "frame_rpy_rad:=${FRAME_RPY_RAD}"
  "takeoff_altitude_m:=${TAKEOFF_ALTITUDE_M}"
  "takeoff_rate_m_s:=${TAKEOFF_RATE_M_S}"
  "hover_duration_s:=${HOVER_DURATION_S}"
  "hover_mode:=HOLD"
  "use_speed_profile:=false"
  "restore_speed_profile_on_exit:=false"
  "max_horizontal_excursion_m:=${MAX_HORIZONTAL_EXCURSION_M}"
)
CAPTURE_CMD=(
  "${SCRIPT_DIR}/capture_position_hover_debug.sh"
  --duration "${CAPTURE_DURATION_S}"
  --drone-id "${DRONE_ID}"
  --mavros-namespace "${MAVROS_NAMESPACE}"
  --output-dir "${CAPTURE_DIR}"
  --extra-topic "${SOURCE_POSE_TOPIC}"
  --extra-topic "${EXTERNAL_POSE_TOPIC}"
)
READY_CMD=(
  "${SCRIPT_DIR}/check_simple_hover_ready.sh"
  --drone-id "${DRONE_ID}"
  --drone-namespace "${DRONE_NAMESPACE}"
  --mavros-namespace "${MAVROS_NAMESPACE}"
  --mocap-namespace "${MOCAP_NAMESPACE}"
  --rigid-body-name "${RIGID_BODY_NAME}"
  --source-pose-topic "${SOURCE_POSE_TOPIC}"
)
START_CMD=(
  "${SCRIPT_DIR}/start_simple_hover_test.sh"
  --drone-id "${DRONE_ID}"
  --drone-namespace "${DRONE_NAMESPACE}"
  --mavros-namespace "${MAVROS_NAMESPACE}"
  --mocap-namespace "${MOCAP_NAMESPACE}"
  --rigid-body-name "${RIGID_BODY_NAME}"
  --source-pose-topic "${SOURCE_POSE_TOPIC}"
)
ABORT_CMD=(
  "${SCRIPT_DIR}/abort_simple_hover_test.sh"
  --drone-id "${DRONE_ID}"
  --drone-namespace "${DRONE_NAMESPACE}"
)

if [[ "${DRY_RUN}" -eq 1 ]]; then
  log "Dry run only. No baseline restore, launch, capture, or service call will run."
  log "Resolved output dir: ${OUTPUT_DIR}"
  if [[ "${RESTORE_BASELINE}" -eq 1 ]]; then
    log "Baseline restore command:"
    print_command "${BASELINE_CMD[@]}"
  else
    log "Baseline restore: skipped"
  fi
  if [[ "${AUTO_START_ROUTER}" -eq 1 ]]; then
    log "mavlink-router autostart:"
    print_command "${MAVLINK_ROUTER_START_SCRIPT}"
  else
    log "mavlink-router autostart: disabled"
  fi
  log "Launch command:"
  print_command "${LAUNCH_CMD[@]}"
  log "Capture command:"
  print_command "${CAPTURE_CMD[@]}"
  log "Readiness command:"
  print_command "${READY_CMD[@]}"
  log "Operator start command:"
  print_command "${START_CMD[@]}"
  log "Abort command:"
  print_command "${ABORT_CMD[@]}"
  exit 0
fi

mkdir -p "${OUTPUT_DIR}"
trap handle_exit EXIT INT TERM

{
  echo "run_id=${RUN_ID}"
  echo "stamp=${STAMP}"
  echo "output_dir=${OUTPUT_DIR}"
  echo "drone_id=${DRONE_ID}"
  echo "drone_namespace=${DRONE_NAMESPACE}"
  echo "mavros_namespace=${MAVROS_NAMESPACE}"
  echo "mocap_namespace=${MOCAP_NAMESPACE}"
  echo "rigid_body_name=${RIGID_BODY_NAME}"
  echo "source_pose_topic=${SOURCE_POSE_TOPIC}"
  echo "frame_rpy_rad=${FRAME_RPY_RAD}"
  echo "takeoff_altitude_m=${TAKEOFF_ALTITUDE_M}"
  echo "hover_duration_s=${HOVER_DURATION_S}"
  echo "capture_duration_s=${CAPTURE_DURATION_S}"
  echo "mavlink_router_start_script=${MAVLINK_ROUTER_START_SCRIPT}"
  echo "mavlink_router_start_timeout_s=${MAVLINK_ROUTER_START_TIMEOUT_S}"
  echo "mavlink_router_autostart=${AUTO_START_ROUTER}"
  echo "restore_baseline=${RESTORE_BASELINE}"
  echo "launch_command="
  print_command "${LAUNCH_CMD[@]}"
  echo "start_command="
  print_command "${START_CMD[@]}"
  echo "abort_command="
  print_command "${ABORT_CMD[@]}"
} > "${OUTPUT_DIR}/run_config.txt"

source_ros_env

if [[ "${RESTORE_BASELINE}" -eq 1 ]]; then
  restore_hover_baseline || fail "Hover baseline restore failed. See ${OUTPUT_DIR}/baseline_restore.log."
else
  warn "Skipping hover baseline restore by request."
  if [[ "${AUTO_START_ROUTER}" -eq 1 ]]; then
    ensure_mavlink_router
  fi
fi

log "Launching position_hover_demo; log: ${LAUNCH_LOG}"
(
  set +e
  "${LAUNCH_CMD[@]}" > "${LAUNCH_LOG}" 2>&1
) &
LAUNCH_PID="$!"
JOB_PIDS+=("${LAUNCH_PID}")

sleep 2

log "Starting debug capture; log: ${CAPTURE_LOG}"
(
  set +e
  "${CAPTURE_CMD[@]}" > "${CAPTURE_LOG}" 2>&1
) &
CAPTURE_PID="$!"
JOB_PIDS+=("${CAPTURE_PID}")

if ! wait_for_ready; then
  fail "Hover stack did not become ready. See ${READY_LOG} and ${LAUNCH_LOG}."
fi

echo ""
log "Ready for supervised start. This script has not started flight."
log "Operator start command:"
print_command "${START_CMD[@]}"
log "Abort command:"
print_command "${ABORT_CMD[@]}"
echo ""

monitor_status=0
if monitor_until_terminal_state; then
  monitor_status=0
else
  monitor_status=$?
fi

if [[ "${monitor_status}" -eq 0 ]]; then
  log "Saved run artifacts under ${OUTPUT_DIR}"
elif [[ "${monitor_status}" -eq 2 ]]; then
  warn "Run artifacts saved under ${OUTPUT_DIR}; sequence ended in ABORT."
else
  fail "Monitoring ended before COMPLETE/ABORT. See ${OUTPUT_DIR}."
fi

stop_background_jobs
trap - EXIT INT TERM
exit "${monitor_status}"
