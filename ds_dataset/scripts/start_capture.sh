#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASET_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_DIR="${DATASET_ROOT}/runtime"
ACTIVE_FILE="${RUNTIME_DIR}/active_capture.env"

mkdir -p "${RUNTIME_DIR}"

find_running_recorders() {
  ps -eo pid=,args= | grep '[r]ecord_realsense_dataset.py' || true
}

if [[ -f "${ACTIVE_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ACTIVE_FILE}"
  if kill -0 "${CAPTURE_PID}" 2>/dev/null; then
    echo "A capture session is already running." >&2
    echo "PID: ${CAPTURE_PID}" >&2
    echo "Session: ${SESSION_NAME}" >&2
    echo "Log: ${LOG_PATH}" >&2
    exit 1
  fi
  rm -f "${ACTIVE_FILE}"
fi

ORPHANED_RECORDERS="$(find_running_recorders)"
if [[ -n "${ORPHANED_RECORDERS}" ]]; then
  echo "A dataset recorder is already running, but the active PID file is stale or missing." >&2
  echo "${ORPHANED_RECORDERS}" >&2
  echo "Run ./stop_capture.sh first, then try again." >&2
  exit 1
fi

TAG="${1:-manual_flight}"
if [[ $# -gt 0 ]]; then
  shift
fi

SAFE_TAG="$(printf '%s' "${TAG}" | tr -cs 'A-Za-z0-9._-' '_' | sed -e 's/^_*//' -e 's/_*$//')"
if [[ -z "${SAFE_TAG}" ]]; then
  SAFE_TAG="manual_flight"
fi

SESSION_NAME="$(date +%Y%m%d_%H%M%S)_${SAFE_TAG}"
SESSION_DIR="${DATASET_ROOT}/sessions/${SESSION_NAME}"
LOG_PATH="${RUNTIME_DIR}/${SESSION_NAME}.log"
STATUS_PATH="${RUNTIME_DIR}/${SESSION_NAME}.status.json"

nohup python3 "${DATASET_ROOT}/scripts/record_realsense_dataset.py" \
  --dataset-root "${DATASET_ROOT}" \
  --runtime-dir "${RUNTIME_DIR}" \
  --tag "${TAG}" \
  --session-name "${SESSION_NAME}" \
  --status-path "${STATUS_PATH}" \
  "$@" \
  </dev/null >"${LOG_PATH}" 2>&1 &

CAPTURE_PID=$!

cat > "${ACTIVE_FILE}" <<EOF
CAPTURE_PID=${CAPTURE_PID}
SESSION_NAME=${SESSION_NAME}
SESSION_DIR=${SESSION_DIR}
LOG_PATH=${LOG_PATH}
STATUS_PATH=${STATUS_PATH}
STARTED_AT=$(date -Iseconds)
EOF

sleep 2

if ! kill -0 "${CAPTURE_PID}" 2>/dev/null; then
  echo "Capture process exited before it finished starting." >&2
  echo "Log: ${LOG_PATH}" >&2
  tail -n 20 "${LOG_PATH}" >&2 || true
  rm -f "${ACTIVE_FILE}"
  exit 1
fi

echo "Started capture session."
echo "PID: ${CAPTURE_PID}"
echo "Session: ${SESSION_NAME}"
echo "Session dir: ${SESSION_DIR}"
echo "Log: ${LOG_PATH}"
echo "Status: ${STATUS_PATH}"
