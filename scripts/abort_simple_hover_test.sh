#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

DRONE_ID="${SIMPLE_HOVER_DRONE_ID:-cdrone4}"
DRONE_NAMESPACE="${SIMPLE_HOVER_DRONE_NAMESPACE:-}"
SERVICE_TIMEOUT_S=8

usage() {
  cat <<'EOF'
Usage:
  scripts/abort_simple_hover_test.sh [options]

Calls the cdrone4 simple-hover abort service. Use this if the hover sequence
needs to be stopped after it has started.

Options:
  --drone-id ID           Drone id. Default: cdrone4
  --drone-namespace NS    Drone namespace. Default: /cdrone/<drone-id>
  --service-timeout SEC   Abort service call timeout. Default: 8
  -h, --help              Show this help.
EOF
}

log() {
  printf '[simple_hover_abort] %s\n' "$*"
}

fail() {
  printf '[simple_hover_abort] ERROR: %s\n' "$*" >&2
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
    --service-timeout)
      SERVICE_TIMEOUT_S="$2"
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

validate_positive_number "--service-timeout" "${SERVICE_TIMEOUT_S}"

DRONE_ID="${DRONE_ID#/}"
DRONE_ID="${DRONE_ID%/}"
[[ -n "${DRONE_ID}" ]] || fail "--drone-id must not be empty."

DRONE_NAMESPACE="$(normalize_ns "${DRONE_NAMESPACE:-/cdrone/${DRONE_ID}}")"
ABORT_SERVICE="$(join_topic "${DRONE_NAMESPACE}" "demo/position_hover_abort")"

source_ros_env

log "Calling abort service: ${ABORT_SERVICE}"
OUTPUT="$(
  timeout "${SERVICE_TIMEOUT_S}s" \
    ros2 service call "${ABORT_SERVICE}" std_srvs/srv/Trigger "{}" 2>&1 || true
)"
printf '%s\n' "${OUTPUT}"

if grep -Eiq 'success[:=][[:space:]]*(true|True)|success=True' <<<"${OUTPUT}"; then
  log "Abort accepted."
  exit 0
fi

fail "Abort service did not report success=true."
