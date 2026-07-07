#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DURATION_S=6
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-}"
REQUEST_STREAM_RATE=""
DISCOVERY_SPIN_TIME="${PX4FLOW_DISCOVERY_SPIN_TIME:-2}"

usage() {
  cat <<'EOF'
Usage:
  scripts/check_px4flow_topics.sh [options]

Bench-check MAVROS optical-flow outputs from a PX4/ARK flow sensor.

Options:
  --mavros-namespace NS       MAVROS namespace. Default: from droneid_config.yaml.
  --duration SEC              Seconds to wait for samples and rate estimates. Default: 6
  --discovery-spin-time SEC   Seconds to wait for ROS discovery list calls. Default: 2
  --request-stream-rate HZ    Ask PX4/MAVROS to stream OPTICAL_FLOW_RAD and OPTICAL_FLOW at HZ.
  -h, --help                  Show this help.

Examples:
  scripts/check_px4flow_topics.sh
  scripts/check_px4flow_topics.sh --duration 10
  scripts/check_px4flow_topics.sh --request-stream-rate 10
  scripts/check_px4flow_topics.sh --mavros-namespace /cdrone/cdrone3/mavros
EOF
}

log() {
  printf '[check_px4flow] %s\n' "$*"
}

warn() {
  printf '[check_px4flow] WARN: %s\n' "$*" >&2
}

fail() {
  printf '[check_px4flow] ERROR: %s\n' "$*" >&2
  exit 1
}

read_drone_config_value() {
  local key="$1"
  python3 - "$REPO_ROOT" "$key" <<'PY' || true
from pathlib import Path
import sys

import yaml

repo_root = Path(sys.argv[1])
key = sys.argv[2]
config_path = repo_root / "ros2" / "src" / "drone_bringup" / "config" / "droneid_config.yaml"
loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
merged = {}
if isinstance(loaded, dict):
    for section_name in ("identity", "network", "mocap", "tracking"):
        section = loaded.get(section_name)
        if isinstance(section, dict):
            merged.update(section)
value = merged.get(key, "")
print("" if value is None else value)
PY
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
  if [[ -f "${REPO_ROOT}/ros2/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${REPO_ROOT}/ros2/install/setup.bash"
  fi

  if [[ "${restore_nounset}" -eq 1 ]]; then
    set -u
  fi

  command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available. Source ROS 2 or run setup_jetson.sh first."
}

ensure_ros_log_dir() {
  if [[ -n "${ROS_LOG_DIR:-}" ]]; then
    mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
    return
  fi

  local default_log_dir="${HOME:-}/.ros/log"
  if [[ -n "${HOME:-}" ]]; then
    mkdir -p "${default_log_dir}" 2>/dev/null || true
    if [[ -w "${default_log_dir}" ]]; then
      return
    fi
  fi

  export ROS_LOG_DIR="${REPO_ROOT}/log/ros_cli"
  mkdir -p "${ROS_LOG_DIR}" 2>/dev/null || true
  warn "~/.ros/log is not writable; using ROS_LOG_DIR=${ROS_LOG_DIR}"
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
    fail "ros2 ${entity} list failed. Check ROS_DOMAIN_ID, RMW_IMPLEMENTATION, and ROS/network permissions."
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

print_graph_hint() {
  local topic_count
  local service_count
  topic_count="$(visible_count "${TOPIC_LIST}")"
  service_count="$(visible_count "${SERVICE_LIST}")"

  warn "Visible ROS graph: ${topic_count} topic(s), ${service_count} service(s)."
  if [[ "${topic_count}" -le 2 ]]; then
    warn "Only ROS bookkeeping topics are visible. MAVROS/bringup is not visible in this terminal."
  fi
  warn "If MAVROS is running in another terminal, verify both terminals use the same ROS_DOMAIN_ID and RMW_IMPLEMENTATION."
}

topic_exists() {
  local topic="$1"
  grep -Fxq -- "${topic}" <<<"${TOPIC_LIST}"
}

service_exists() {
  local service="$1"
  grep -Fxq -- "${service}" <<<"${SERVICE_LIST}"
}

validate_positive_integer() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[1-9][0-9]*$ ]] || fail "${name} must be a positive integer, got '${value}'."
}

validate_positive_number() {
  local name="$1"
  local value="$2"
  [[ "${value}" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail "${name} must be a positive number, got '${value}'."
  python3 - "$name" "$value" <<'PY'
import sys

name = sys.argv[1]
value = float(sys.argv[2])
if value <= 0.0:
    raise SystemExit(f"{name} must be greater than zero.")
PY
}

request_stream() {
  local service="${MAVROS_NAMESPACE}/set_message_interval"
  local message_id="$1"
  local message_name="$2"

  if ! service_exists "${service}"; then
    warn "MAVROS service ${service} is not available; cannot request ${message_name} stream."
    return 1
  fi

  log "Requesting ${message_name} (${message_id}) at ${REQUEST_STREAM_RATE} Hz via ${service}"
  local output
  output="$(
    timeout 8 ros2 service call \
      "${service}" \
      mavros_msgs/srv/MessageInterval \
      "{message_id: ${message_id}, message_rate: ${REQUEST_STREAM_RATE}}" \
      2>&1 || true
  )"
  printf '%s\n' "${output}" | sed 's/^/  /'

  if ! grep -Eq 'success[:=][[:space:]]*(true|True)' <<<"${output}"; then
    warn "MAVROS did not confirm ${message_name} stream request success."
    return 1
  fi
}

check_mavros_connection() {
  local state_topic="${MAVROS_NAMESPACE}/state"

  log "Checking MAVROS state topic ${state_topic}"
  if ! topic_exists "${state_topic}"; then
    warn "${state_topic} is missing. Start MAVROS with: ros2 launch drone_bringup drone.launch.py"
    print_graph_hint
    return 1
  fi

  local connected
  connected="$(
    timeout 8 ros2 topic echo --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" --once --field connected "${state_topic}" 2>/dev/null \
      | tr '[:upper:]' '[:lower:]' \
      | sed -e 's/^[[:space:]]*//' -e 's/[[:space:]]*$//' \
      || true
  )"
  if grep -Fxq "true" <<<"${connected}"; then
    log "MAVROS reports connected=true"
    return 0
  fi

  warn "MAVROS state topic exists, but connected=true was not observed."
  return 1
}

inspect_topic() {
  local topic="$1"
  local expected_type="$2"
  local saw_live_output=0

  echo ""
  log "Inspecting ${topic}"

  if ! topic_exists "${topic}"; then
    echo "  status: missing"
    return 1
  fi

  local topic_type
  topic_type="$(ros2 topic type --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" "${topic}" 2>/dev/null || true)"
  echo "  status: present"
  echo "  type: ${topic_type:-unknown}"

  if [[ -n "${expected_type}" && "${topic_type}" != "${expected_type}" ]]; then
    warn "${topic} type is '${topic_type:-unknown}', expected '${expected_type}'."
  fi

  echo "  sample:"
  local sample
  sample="$(timeout "${DURATION_S}" ros2 topic echo --no-daemon --spin-time "${DISCOVERY_SPIN_TIME}" --once "${topic}" 2>/dev/null || true)"
  if [[ -n "${sample}" ]]; then
    printf '%s\n' "${sample}" | sed 's/^/    /'
    saw_live_output=1
  else
    echo "    no sample received within ${DURATION_S}s"
  fi

  echo "  rate (${DURATION_S}s window):"
  local hz_output
  hz_output="$(timeout "$((DURATION_S + 1))" ros2 topic hz "${topic}" 2>/dev/null || true)"
  if [[ -n "${hz_output}" ]]; then
    printf '%s\n' "${hz_output}" | tail -n 5 | sed 's/^/    /'
  else
    echo "    no rate estimate received"
  fi

  if [[ "${saw_live_output}" -eq 1 ]] && grep -Fq "average rate" <<<"${hz_output}"; then
    return 0
  fi
  return 1
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mavros-namespace)
      [[ $# -ge 2 ]] || fail "--mavros-namespace requires a value."
      MAVROS_NAMESPACE="$2"
      shift 2
      ;;
    --duration)
      [[ $# -ge 2 ]] || fail "--duration requires a value."
      DURATION_S="$2"
      shift 2
      ;;
    --discovery-spin-time)
      [[ $# -ge 2 ]] || fail "--discovery-spin-time requires a value."
      DISCOVERY_SPIN_TIME="$2"
      shift 2
      ;;
    --request-stream-rate)
      [[ $# -ge 2 ]] || fail "--request-stream-rate requires a value."
      REQUEST_STREAM_RATE="$2"
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

validate_positive_integer "--duration" "${DURATION_S}"
validate_positive_number "--discovery-spin-time" "${DISCOVERY_SPIN_TIME}"
if [[ -n "${REQUEST_STREAM_RATE}" ]]; then
  validate_positive_number "--request-stream-rate" "${REQUEST_STREAM_RATE}"
fi

if [[ -z "${MAVROS_NAMESPACE}" ]]; then
  MAVROS_NAMESPACE="$(read_drone_config_value mavros_namespace)"
fi
if [[ -z "${MAVROS_NAMESPACE}" ]]; then
  DRONE_ID="$(read_drone_config_value drone_id)"
  if [[ -n "${DRONE_ID}" ]]; then
    MAVROS_NAMESPACE="/cdrone/${DRONE_ID}/mavros"
  else
    MAVROS_NAMESPACE="/mavros"
  fi
fi
MAVROS_NAMESPACE="$(normalize_ns "${MAVROS_NAMESPACE}")"

source_ros_env
ensure_ros_log_dir

log "Using MAVROS namespace: ${MAVROS_NAMESPACE}"
log "Sample/rate wait duration: ${DURATION_S}s"
log "ROS discovery spin time: ${DISCOVERY_SPIN_TIME}s"

TOPIC_LIST="$(ros2_list_visible_names topic)"
SERVICE_LIST="$(ros2_list_visible_names service)"

check_mavros_connection || true

if [[ -n "${REQUEST_STREAM_RATE}" ]]; then
  request_stream 106 "OPTICAL_FLOW_RAD" || true
  request_stream 100 "OPTICAL_FLOW" || true
  TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
fi

TOPICS=(
  "${MAVROS_NAMESPACE}/px4flow/raw/optical_flow_rad"
  "${MAVROS_NAMESPACE}/px4flow/ground_distance"
  "${MAVROS_NAMESPACE}/optical_flow/raw/optical_flow"
  "${MAVROS_NAMESPACE}/optical_flow/ground_distance"
)

EXPECTED_TYPES=(
  "mavros_msgs/msg/OpticalFlowRad"
  "sensor_msgs/msg/Range"
  "mavros_msgs/msg/OpticalFlow"
  "sensor_msgs/msg/Range"
)

LIVE_TOPIC_COUNT=0
for index in "${!TOPICS[@]}"; do
  if inspect_topic "${TOPICS[$index]}" "${EXPECTED_TYPES[$index]}"; then
    LIVE_TOPIC_COUNT=$((LIVE_TOPIC_COUNT + 1))
  fi
done

echo ""
if [[ "${LIVE_TOPIC_COUNT}" -gt 0 ]]; then
  log "PASS: ${LIVE_TOPIC_COUNT} optical-flow/range topic(s) produced a sample and rate estimate."
  exit 0
fi

fail "No checked optical-flow/range topic produced both a sample and rate estimate."
