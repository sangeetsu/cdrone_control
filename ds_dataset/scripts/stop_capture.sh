#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATASET_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUNTIME_DIR="${DATASET_ROOT}/runtime"
ACTIVE_FILE="${RUNTIME_DIR}/active_capture.env"

find_running_recorders() {
  ps -eo pid=,args= | grep '[r]ecord_realsense_dataset.py' || true
}

CAPTURE_PID=""
SESSION_NAME="unknown"
SESSION_DIR="unknown"
LOG_PATH="unknown"
STATUS_PATH=""

if [[ -f "${ACTIVE_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${ACTIVE_FILE}"
fi

if [[ -n "${CAPTURE_PID}" ]] && ! kill -0 "${CAPTURE_PID}" 2>/dev/null; then
  CAPTURE_PID=""
fi

if [[ -z "${CAPTURE_PID}" ]]; then
  mapfile -t RUNNING_LINES < <(find_running_recorders)
  if [[ ${#RUNNING_LINES[@]} -eq 0 ]]; then
    echo "No running dataset recorder found."
    rm -f "${ACTIVE_FILE}"
    exit 0
  fi
  if [[ ${#RUNNING_LINES[@]} -gt 1 ]]; then
    echo "Multiple dataset recorder processes are running. Stop them manually:" >&2
    printf '%s\n' "${RUNNING_LINES[@]}" >&2
    exit 1
  fi
  CAPTURE_PID="$(printf '%s\n' "${RUNNING_LINES[0]}" | awk '{print $1}')"
  echo "Stopping recorder process discovered from process list."
fi

kill -INT "${CAPTURE_PID}"

for _ in $(seq 1 30); do
  if ! kill -0 "${CAPTURE_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done

if kill -0 "${CAPTURE_PID}" 2>/dev/null; then
  echo "Capture is still running after 30 seconds." >&2
  echo "Inspect log: ${LOG_PATH}" >&2
  exit 1
fi

rm -f "${ACTIVE_FILE}"

echo "Stopped capture session."
echo "Session: ${SESSION_NAME}"
echo "Session dir: ${SESSION_DIR}"
echo "Log: ${LOG_PATH}"
if [[ -n "${STATUS_PATH}" && -f "${STATUS_PATH}" ]]; then
  echo "Status: ${STATUS_PATH}"
fi
