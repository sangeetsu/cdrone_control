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

if [[ -n "${CAPTURE_PID}" ]] && kill -0 "${CAPTURE_PID}" 2>/dev/null; then
  echo "Capture is running."
  echo "PID: ${CAPTURE_PID}"
  echo "Session: ${SESSION_NAME}"
  echo "Session dir: ${SESSION_DIR}"
  echo "Log: ${LOG_PATH}"
  if [[ -n "${STATUS_PATH}" && -f "${STATUS_PATH}" ]]; then
    python3 -c '
import json, sys
path = sys.argv[1]
with open(path, "r", encoding="utf-8") as handle:
    data = json.load(handle)
print(f"State: {data.get(\"state\", \"unknown\")}")
print(f"Saved frames: {data.get(\"total_frames_saved\", 0)}")
print(f"Seen frames: {data.get(\"total_frames_seen\", 0)}")
print(f"Skipped frames: {data.get(\"total_frames_skipped\", 0)}")
print(f"Last saved at: {data.get(\"last_saved_at\")}")
' "${STATUS_PATH}"
  fi
  exit 0
fi

mapfile -t RUNNING_LINES < <(find_running_recorders)
if [[ ${#RUNNING_LINES[@]} -eq 0 ]]; then
  echo "No active capture session."
  exit 0
fi

echo "A dataset recorder is running, but the active PID file is stale or missing."
printf '%s\n' "${RUNNING_LINES[@]}"
exit 1
