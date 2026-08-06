#!/usr/bin/env bash
set -Eeuo pipefail

DRONE_ID="${DRONE_ID:-cdrone3}"
RIGID_BODY_NAME="${RIGID_BODY_NAME:-RigidBody4}"
TAKEOFF_ALTITUDE_M="${TAKEOFF_ALTITUDE_M:-0.6}"
TAKEOFF_RATE_M_S="${TAKEOFF_RATE_M_S:-0.3}"
HOVER_DURATION_S="${HOVER_DURATION_S:-5.0}"
MAX_HORIZONTAL_EXCURSION_M="${MAX_HORIZONTAL_EXCURSION_M:-0.5}"
GLOBAL_ORIGIN_LAT="${GLOBAL_ORIGIN_LAT:-0.0}"
GLOBAL_ORIGIN_LON="${GLOBAL_ORIGIN_LON:-0.0}"
GLOBAL_ORIGIN_ALT="${GLOBAL_ORIGIN_ALT:-17.1637}"
HOME_X="${HOME_X:-0.0}"
HOME_Y="${HOME_Y:-0.0}"
HOME_Z="${HOME_Z:-0.0}"
HOME_APPROACH_Z="${HOME_APPROACH_Z:-1.0}"
START_TIMEOUT_S="${START_TIMEOUT_S:-90}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NS="/cdrone/${DRONE_ID}"
MAVROS_NS="${NS}/mavros"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)"
LOG_DIR="${REPO_ROOT}/temp_outputs/cdrone3_hover_${RUN_ID}"
LAUNCH_LOG="${LOG_DIR}/position_hover_launch.log"
LAUNCH_PID=""
STARTED=0

mkdir -p "${LOG_DIR}"

log() {
  printf '[hover-run] %s\n' "$*"
}

die() {
  log "ERROR: $*"
  exit 1
}

source_ros() {
  set +u
  # shellcheck disable=SC1091
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  [[ -f "${REPO_ROOT}/install/setup.bash" ]] && source "${REPO_ROOT}/install/setup.bash"
  set -u
}

safe_stop() {
  set +e
  if [[ "${STARTED}" == "1" ]]; then
    log "Abort requested; sending demo abort and AUTO.LAND."
    ros2 service call "${NS}/demo/position_hover_abort" std_srvs/srv/Trigger "{}" >/dev/null 2>&1
    ros2 service call "${MAVROS_NS}/set_mode" mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.LAND'}" >/dev/null 2>&1
  fi
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    log "Stopping launch process ${LAUNCH_PID}."
    kill -INT "${LAUNCH_PID}" 2>/dev/null
    wait "${LAUNCH_PID}" 2>/dev/null
  fi
}
trap safe_stop INT TERM

wait_for_service() {
  local service_name="$1"
  local deadline=$((SECONDS + START_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if ros2 service type "${service_name}" >/dev/null 2>&1; then
      log "Service ready: ${service_name}"
      return 0
    fi
    sleep 1
  done
  die "Timed out waiting for service: ${service_name}"
}

capture_once() {
  local label="$1"
  local output_file="$2"
  shift 2
  log "Checking ${label}..."
  if timeout 8s "$@" >"${output_file}" 2>&1; then
    sed -n '1,40p' "${output_file}"
    return 0
  fi
  sed -n '1,80p' "${output_file}" || true
  return 1
}

call_trigger() {
  local service_name="$1"
  local output_file="$2"
  ros2 service call "${service_name}" std_srvs/srv/Trigger "{}" >"${output_file}" 2>&1
  sed -n '1,80p' "${output_file}"
  grep -q "success=True" "${output_file}"
}

set_mode() {
  local mode="$1"
  local output_file="${LOG_DIR}/set_mode_${mode}.txt"
  log "Requesting PX4 mode ${mode}..."
  ros2 service call "${MAVROS_NS}/set_mode" mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: '${mode}'}" >"${output_file}" 2>&1
  sed -n '1,80p' "${output_file}"
  grep -q "mode_sent=True" "${output_file}"
}

publish_reference() {
  log "Publishing indoor global origin and home position."
  ros2 topic pub --once "${MAVROS_NS}/global_position/set_gp_origin" geographic_msgs/msg/GeoPointStamped \
    "{header: {frame_id: 'map'}, position: {latitude: ${GLOBAL_ORIGIN_LAT}, longitude: ${GLOBAL_ORIGIN_LON}, altitude: ${GLOBAL_ORIGIN_ALT}}}" \
    >"${LOG_DIR}/publish_origin.txt" 2>&1 || true

  ros2 topic pub --once "${MAVROS_NS}/home_position/set" mavros_msgs/msg/HomePosition \
    "{header: {frame_id: 'map'}, geo: {latitude: ${GLOBAL_ORIGIN_LAT}, longitude: ${GLOBAL_ORIGIN_LON}, altitude: ${GLOBAL_ORIGIN_ALT}}, position: {x: ${HOME_X}, y: ${HOME_Y}, z: ${HOME_Z}}, orientation: {w: 1.0}, approach: {z: ${HOME_APPROACH_Z}}}" \
    >"${LOG_DIR}/publish_home.txt" 2>&1 || true
}

wait_for_reference_setup() {
  local deadline=$((SECONDS + START_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if rg -q "Indoor global origin and home position are confirmed by MAVROS" "${LAUNCH_LOG}" 2>/dev/null; then
      log "Indoor global origin and home position confirmed by setup node."
      return 0
    fi
    publish_reference
    sleep 2
  done

  log "Reference setup did not confirm. Recent setup lines:"
  rg -n "drone_setup|global origin|home position|FCU disconnected|FCU connected" "${LAUNCH_LOG}" | tail -n 40 || true
  return 1
}

require_state_field() {
  local pattern="$1"
  local description="$2"
  local output_file="${LOG_DIR}/state_check.txt"
  capture_once "MAVROS state" "${output_file}" \
    ros2 topic echo --once --qos-reliability best_effort "${MAVROS_NS}/state" \
    || die "Could not read MAVROS state."
  grep -q "${pattern}" "${output_file}" || die "State check failed: expected ${description}. See ${output_file}"
}

source_ros
cd "${REPO_ROOT}"

log "Starting hover stack for ${DRONE_ID} / ${RIGID_BODY_NAME}."
log "Log file: ${LAUNCH_LOG}"
ros2 launch drone_bringup position_hover_demo.launch.py \
  pose_source:=optitrack \
  rigid_body_name:="${RIGID_BODY_NAME}" \
  enable_pose_debug:=true \
  takeoff_altitude_m:="${TAKEOFF_ALTITUDE_M}" \
  takeoff_rate_m_s:="${TAKEOFF_RATE_M_S}" \
  hover_duration_s:="${HOVER_DURATION_S}" \
  max_horizontal_excursion_m:="${MAX_HORIZONTAL_EXCURSION_M}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

sleep 3
wait_for_service "${MAVROS_NS}/set_mode"
wait_for_service "${NS}/demo/position_hover_start"
wait_for_service "${NS}/demo/position_hover_abort"

publish_reference
wait_for_reference_setup || die "Global origin/home setup is still missing."

capture_once "home position" "${LOG_DIR}/home_position.txt" \
  ros2 topic echo --once "${MAVROS_NS}/home_position/home" \
  || log "Warning: one-shot home_position echo missed a sample; continuing because setup node confirmed home/origin."

capture_once "local position" "${LOG_DIR}/local_position.txt" \
  ros2 topic echo --once "${MAVROS_NS}/local_position/pose" \
  || die "Local position is missing. Check mocap / ${RIGID_BODY_NAME}."

set_mode "AUTO.LOITER" || die "PX4 did not accept AUTO.LOITER."
sleep 1
require_state_field "mode: AUTO.LOITER" "mode: AUTO.LOITER"
require_state_field "guided: true" "guided: true"

log "Starting position hover demo. Be ready to hit Ctrl-C here to abort/land."
STARTED=1
call_trigger "${NS}/demo/position_hover_start" "${LOG_DIR}/start_response.txt" \
  || die "Start service rejected request. See ${LOG_DIR}/start_response.txt and ${LAUNCH_LOG}"

log "Monitoring demo state. The script exits after LAND/IDLE/ABORT or timeout."
deadline=$((SECONDS + START_TIMEOUT_S))
last_state=""
while (( SECONDS < deadline )); do
  state_file="${LOG_DIR}/demo_state_latest.txt"
  if timeout 3s ros2 topic echo --once "${NS}/demo/position_hover_state" >"${state_file}" 2>&1; then
    state="$(awk '/data:/ {print $2; exit}' "${state_file}")"
    if [[ -n "${state}" && "${state}" != "${last_state}" ]]; then
      log "Demo state: ${state}"
      last_state="${state}"
    fi
    case "${state}" in
      ABORT)
        log "Demo aborted. Relevant launch lines:"
        rg -n "Aborting demo|State .*ABORT|Arm request|rejected|failed|Preflight|FCU:" "${LAUNCH_LOG}" || true
        exit 2
        ;;
      IDLE|LAND)
        log "Demo reached ${state}."
        exit 0
        ;;
    esac
  fi
  sleep 1
done

die "Timed out while monitoring hover demo. Logs are in ${LOG_DIR}"
