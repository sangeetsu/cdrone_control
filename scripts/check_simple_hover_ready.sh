#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DRONE_ID="${SIMPLE_HOVER_DRONE_ID:-cdrone4}"
DRONE_NAMESPACE="${SIMPLE_HOVER_DRONE_NAMESPACE:-}"
MAVROS_NAMESPACE="${SIMPLE_HOVER_MAVROS_NAMESPACE:-}"
MOCAP_NAMESPACE="${SIMPLE_HOVER_MOCAP_NAMESPACE:-}"
RIGID_BODY_NAME="${SIMPLE_HOVER_RIGID_BODY_NAME:-RigidBody4}"
SOURCE_POSE_TOPIC="${SIMPLE_HOVER_SOURCE_POSE_TOPIC:-}"
DISCOVERY_SPIN_TIME="${SIMPLE_HOVER_DISCOVERY_SPIN_TIME:-2}"
SAMPLE_TIMEOUT_S="${SIMPLE_HOVER_SAMPLE_TIMEOUT_S:-6}"
RATE_WINDOW_S="${SIMPLE_HOVER_RATE_WINDOW_S:-5}"
MIN_LOCAL_POSE_HZ="${SIMPLE_HOVER_MIN_LOCAL_POSE_HZ:-10}"
MIN_SOURCE_POSE_HZ="${SIMPLE_HOVER_MIN_SOURCE_POSE_HZ:-20}"
MIN_VISION_POSE_HZ="${SIMPLE_HOVER_MIN_VISION_POSE_HZ:-10}"
SKIP_RATE_CHECKS=0

usage() {
  cat <<'EOF'
Usage:
  scripts/check_simple_hover_ready.sh [options]

Checks whether the cdrone4/RigidBody4 simple hover stack is ready for the
operator to call the explicit start service. This does not arm or start flight.

Options:
  --drone-id ID              Drone id. Default: cdrone4
  --drone-namespace NS       Drone namespace. Default: /cdrone/<drone-id>
  --mavros-namespace NS      MAVROS namespace. Default: /cdrone/<drone-id>/mavros
  --mocap-namespace NS       Mocap namespace. Default: /cdrone/<drone-id>/vrpn_mocap
  --rigid-body-name NAME     Mocap rigid body. Default: RigidBody4
  --source-pose-topic TOPIC  Source pose topic. Default: <mocap-ns>/<rigid-body>/pose
  --discovery-spin-time SEC  ROS discovery wait. Default: 2
  --sample-timeout SEC       Per-topic sample wait. Default: 6
  --rate-window SEC          ros2 topic hz window. Default: 5
  --min-local-pose-hz HZ     Local-pose minimum rate. Default: 10
  --min-source-pose-hz HZ    Source-pose minimum rate. Default: 20
  --min-vision-pose-hz HZ    Vision-pose minimum rate. Default: 10
  --skip-rate-checks         Check presence/samples only, not rates.
  -h, --help                 Show this help.
EOF
}

log() {
  printf '[simple_hover_ready] %s\n' "$*"
}

warn() {
  printf '[simple_hover_ready] WARN: %s\n' "$*" >&2
}

fail() {
  printf '[simple_hover_ready] ERROR: %s\n' "$*" >&2
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

ros2_list_visible_names() {
  local entity="$1"
  local output
  local status=0

  output="$(
    ros2 "${entity}" list --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" 2>&1
  )" || status=$?

  if [[ "${status}" -ne 0 ]]; then
    printf '%s\n' "${output}" | sed 's/^/  /' >&2
    fail "ros2 ${entity} list failed. Check ROS_DOMAIN_ID, RMW_IMPLEMENTATION, and ROS network setup."
  fi

  if grep -Eiq 'PermissionError|Operation not permitted|TRANSPORT_UDP|failed to register|Read-only file system|getifaddrs' <<<"${output}"; then
    warn "ROS ${entity} discovery reported transport/logging errors:"
    printf '%s\n' "${output}" | sed 's/^/  /' >&2
  fi

  printf '%s\n' "${output}" | sed -n '/^\//p'
}

visible_count() {
  local names="$1"
  grep -Ec '^/' <<<"${names}" || true
}

topic_exists() {
  local topic="$1"
  grep -Fxq -- "${topic}" <<<"${TOPIC_LIST}"
}

service_exists() {
  local service="$1"
  grep -Fxq -- "${service}" <<<"${SERVICE_LIST}"
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

read_field() {
  local topic="$1"
  local field="$2"
  local label="$3"
  local output
  local value

  topic_exists "${topic}" || fail "${label} topic is missing: ${topic}"
  output="$(
    timeout "${SAMPLE_TIMEOUT_S}s" \
      ros2 topic echo --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" \
      --once --field "${field}" "${topic}" 2>/dev/null || true
  )"
  value="$(field_value_from_output "${field}" "${output}")"
  [[ -n "${value}" ]] || fail "No ${field} sample received from ${label} topic ${topic} within ${SAMPLE_TIMEOUT_S}s."
  printf '%s' "${value}"
}

sample_topic() {
  local topic="$1"
  local label="$2"
  local output

  topic_exists "${topic}" || fail "${label} topic is missing: ${topic}"
  output="$(
    timeout "${SAMPLE_TIMEOUT_S}s" \
      ros2 topic echo --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" \
      --once "${topic}" 2>/dev/null || true
  )"
  [[ -n "${output}" ]] || fail "No sample received from ${label} topic ${topic} within ${SAMPLE_TIMEOUT_S}s."
  log "Sample OK: ${label} (${topic})"
}

expect_bool_field() {
  local topic="$1"
  local field="$2"
  local expected="$3"
  local label="$4"
  local value

  value="$(read_field "${topic}" "${field}" "${label}")"
  value="$(tr '[:upper:]' '[:lower:]' <<<"${value}")"
  expected="$(tr '[:upper:]' '[:lower:]' <<<"${expected}")"
  if [[ "${value}" != "${expected}" ]]; then
    fail "${label} ${field} expected ${expected}, got ${value}."
  fi
  log "Field OK: ${label} ${field}=${value}"
}

expect_string_field() {
  local topic="$1"
  local field="$2"
  local expected="$3"
  local label="$4"
  local value

  value="$(read_field "${topic}" "${field}" "${label}")"
  if [[ "${value}" != "${expected}" ]]; then
    fail "${label} ${field} expected ${expected}, got ${value}."
  fi
  log "Field OK: ${label} ${field}=${value}"
}

expect_companion_active() {
  local topic="$1"
  local value

  value="$(read_field "${topic}" "state" "companion status")"
  case "${value}" in
    4|ACTIVE|active|MAV_STATE_ACTIVE)
      log "Field OK: companion status state=${value}"
      ;;
    *)
      fail "companion status expected MAV_STATE_ACTIVE (4), got ${value}."
      ;;
  esac
}

measure_rate_hz() {
  local topic="$1"
  local output

  output="$(
    timeout --signal=INT --kill-after=2s "${RATE_WINDOW_S}s" \
      ros2 topic hz "${topic}" 2>/dev/null || true
  )"
  sed -nE 's/.*average rate:[[:space:]]*([0-9]+([.][0-9]+)?).*/\1/p' \
    <<<"${output}" | tail -n 1
}

expect_rate_at_least() {
  local topic="$1"
  local min_rate="$2"
  local label="$3"
  local observed_rate

  observed_rate="$(measure_rate_hz "${topic}")"
  [[ -n "${observed_rate}" ]] || fail "No rate estimate for ${label} topic ${topic}."
  python3 - "$label" "$observed_rate" "$min_rate" <<'PY'
import sys

label = sys.argv[1]
observed = float(sys.argv[2])
minimum = float(sys.argv[3])
if observed < minimum:
    raise SystemExit(f"{label} rate too low: observed {observed:.2f} Hz, minimum {minimum:.2f} Hz")
print(f"[simple_hover_ready] Rate OK: {label} {observed:.2f} Hz >= {minimum:.2f} Hz")
PY
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
    --discovery-spin-time)
      DISCOVERY_SPIN_TIME="$2"
      shift 2
      ;;
    --sample-timeout)
      SAMPLE_TIMEOUT_S="$2"
      shift 2
      ;;
    --rate-window)
      RATE_WINDOW_S="$2"
      shift 2
      ;;
    --min-local-pose-hz)
      MIN_LOCAL_POSE_HZ="$2"
      shift 2
      ;;
    --min-source-pose-hz)
      MIN_SOURCE_POSE_HZ="$2"
      shift 2
      ;;
    --min-vision-pose-hz)
      MIN_VISION_POSE_HZ="$2"
      shift 2
      ;;
    --skip-rate-checks)
      SKIP_RATE_CHECKS=1
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

validate_positive_number "--discovery-spin-time" "${DISCOVERY_SPIN_TIME}"
validate_positive_number "--sample-timeout" "${SAMPLE_TIMEOUT_S}"
validate_positive_number "--rate-window" "${RATE_WINDOW_S}"
validate_positive_number "--min-local-pose-hz" "${MIN_LOCAL_POSE_HZ}"
validate_positive_number "--min-source-pose-hz" "${MIN_SOURCE_POSE_HZ}"
validate_positive_number "--min-vision-pose-hz" "${MIN_VISION_POSE_HZ}"

DRONE_ID="${DRONE_ID#/}"
DRONE_ID="${DRONE_ID%/}"
[[ -n "${DRONE_ID}" ]] || fail "--drone-id must not be empty."

DRONE_NAMESPACE="$(normalize_ns "${DRONE_NAMESPACE:-/cdrone/${DRONE_ID}}")"
MAVROS_NAMESPACE="$(normalize_ns "${MAVROS_NAMESPACE:-/cdrone/${DRONE_ID}/mavros}")"
MOCAP_NAMESPACE="$(normalize_ns "${MOCAP_NAMESPACE:-/cdrone/${DRONE_ID}/vrpn_mocap}")"
SOURCE_POSE_TOPIC="${SOURCE_POSE_TOPIC:-$(join_topic "${MOCAP_NAMESPACE}" "${RIGID_BODY_NAME}/pose")}"

DEMO_STATE_TOPIC="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_state")"
DEMO_START_SERVICE="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_start")"
DEMO_ABORT_SERVICE="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_abort")"
MAVROS_STATE_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "state")"
LOCAL_POSE_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "local_position/pose")"
HOME_POSITION_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "home_position/home")"
GLOBAL_ORIGIN_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "global_position/gp_origin")"
VISION_POSE_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "vision_pose/pose")"
COMPANION_STATUS_TOPIC="$(join_topic "${MAVROS_NAMESPACE}" "companion_process/status")"

source_ros_env

log "Checking simple hover readiness for ${DRONE_ID} / ${RIGID_BODY_NAME}"
log "MAVROS namespace: ${MAVROS_NAMESPACE}"
log "Source pose topic: ${SOURCE_POSE_TOPIC}"

TOPIC_LIST="$(ros2_list_visible_names topic)"
SERVICE_LIST="$(ros2_list_visible_names service)"
TOPIC_COUNT="$(visible_count "${TOPIC_LIST}")"
SERVICE_COUNT="$(visible_count "${SERVICE_LIST}")"

if [[ "${TOPIC_COUNT}" -le 2 || "${SERVICE_COUNT}" -eq 0 ]]; then
  fail "ROS graph visibility is too sparse: topics=${TOPIC_COUNT}, services=${SERVICE_COUNT}."
fi
log "ROS graph visible: topics=${TOPIC_COUNT}, services=${SERVICE_COUNT}"

for service in \
  "${DEMO_START_SERVICE}" \
  "${DEMO_ABORT_SERVICE}" \
  "$(join_topic "${MAVROS_NAMESPACE}" "set_mode")" \
  "$(join_topic "${MAVROS_NAMESPACE}" "cmd/arming")" \
  "$(join_topic "${MAVROS_NAMESPACE}" "param/get_parameters")" \
  "$(join_topic "${MAVROS_NAMESPACE}" "param/pull")" \
  "$(join_topic "${MAVROS_NAMESPACE}" "param/set_parameters")"; do
  service_exists "${service}" || fail "Required service is missing: ${service}"
  log "Service OK: ${service}"
done

expect_bool_field "${MAVROS_STATE_TOPIC}" "connected" "true" "MAVROS state"
expect_bool_field "${MAVROS_STATE_TOPIC}" "armed" "false" "MAVROS state"
expect_string_field "${DEMO_STATE_TOPIC}" "data" "IDLE" "demo state"

sample_topic "${LOCAL_POSE_TOPIC}" "local pose"
sample_topic "${HOME_POSITION_TOPIC}" "home position"
sample_topic "${GLOBAL_ORIGIN_TOPIC}" "global origin"
sample_topic "${SOURCE_POSE_TOPIC}" "source pose"
sample_topic "${VISION_POSE_TOPIC}" "vision pose"
expect_companion_active "${COMPANION_STATUS_TOPIC}"

if [[ "${SKIP_RATE_CHECKS}" -eq 0 ]]; then
  expect_rate_at_least "${LOCAL_POSE_TOPIC}" "${MIN_LOCAL_POSE_HZ}" "local pose"
  expect_rate_at_least "${SOURCE_POSE_TOPIC}" "${MIN_SOURCE_POSE_HZ}" "source pose"
  expect_rate_at_least "${VISION_POSE_TOPIC}" "${MIN_VISION_POSE_HZ}" "vision pose"
else
  warn "Skipping rate checks by request."
fi

log "PASS: simple hover stack is ready for the supervised start service."
