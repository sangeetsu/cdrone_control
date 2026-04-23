#!/usr/bin/env bash

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"

read_drone_config_value() {
  local key="$1"
  python3 - "$REPO_ROOT" "$key" <<'PY'
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

DURATION_S=120
DRONE_ID="$(read_drone_config_value drone_id)"
MAVROS_NAMESPACE="$(read_drone_config_value mavros_namespace)"
if [[ -z "${DRONE_ID}" ]]; then
  echo "drone_id is missing from ros2/src/drone_bringup/config/droneid_config.yaml" >&2
  exit 1
fi
MAVROS_NAMESPACE="${MAVROS_NAMESPACE:-/cdrone/${DRONE_ID}/mavros}"
OUTPUT_DIR=""
CAPTURE_START_EPOCH_S="$(date +%s)"

declare -a EXTRA_TOPICS=()
declare -a JOB_PIDS=()
declare -a DEMO_LOG_FILES=()
declare -a TAILED_LOG_KEYS=()

SUMMARY_WRITTEN=0
STOPPED_JOBS=0

usage() {
  cat <<'EOF'
Usage:
  scripts/capture_position_hover_debug.sh [options]

Observer-side capture helper for the position hover demo.
Run this while the demo stack is already up, then let the operator trigger the demo.

Options:
  --duration SEC          Capture duration before auto-stop. Default: 120
  --drone-id ID           Drone id used for /cdrone/<id>/... topics. Default: from droneid_config.yaml
  --mavros-namespace NS   MAVROS namespace. Default: from droneid_config.yaml
  --output-dir PATH       Capture directory. Default: temp_outputs/position_hover_capture_<stamp>
  --extra-topic TOPIC     Extra topic to add to the rosbag. Can be repeated.
  --help                  Show this help.

Examples:
  scripts/capture_position_hover_debug.sh
  scripts/capture_position_hover_debug.sh --duration 90
  scripts/capture_position_hover_debug.sh --extra-topic /vrpn_mocap/RigidBody4/pose
EOF
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
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
  fi

  if [[ -f "${HOME}/.bashrc" ]]; then
    # shellcheck disable=SC1091
    source "${HOME}/.bashrc"
  fi

  if [[ -f "${REPO_ROOT}/ros2/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/ros2/install/setup.bash"
  fi
  set -u

  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 is not available after sourcing the environment." >&2
    exit 1
  fi
}

stop_background_jobs() {
  if [[ "${STOPPED_JOBS}" -eq 1 ]]; then
    return
  fi
  STOPPED_JOBS=1

  signal_process_tree() {
    local pid="$1"
    local signal_name="$2"
    local child

    if [[ -z "${pid}" ]]; then
      return
    fi

    while IFS= read -r child; do
      [[ -z "${child}" ]] && continue
      signal_process_tree "${child}" "${signal_name}"
    done < <(pgrep -P "${pid}" || true)

    kill "-${signal_name}" "${pid}" >/dev/null 2>&1 || true
  }

  for pid in "${JOB_PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      signal_process_tree "${pid}" "INT"
    fi
  done

  sleep 1

  for pid in "${JOB_PIDS[@]:-}"; do
    if kill -0 "${pid}" >/dev/null 2>&1; then
      signal_process_tree "${pid}" "TERM"
    fi
  done

  for pid in "${JOB_PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
}

generate_summary() {
  if [[ "${SUMMARY_WRITTEN}" -eq 1 ]]; then
    return
  fi
  SUMMARY_WRITTEN=1

  python3 - "${OUTPUT_DIR}" "${DRONE_ID}" "${MAVROS_NAMESPACE}" "${CAPTURE_START_EPOCH_S}" <<'PY'
import sqlite3
import sys
from pathlib import Path

output_dir = Path(sys.argv[1])
drone_id = sys.argv[2]
mavros_ns = sys.argv[3].rstrip("/")
capture_start_epoch_s = float(sys.argv[4])

demo_state_topic = f"/cdrone/{drone_id}/demo/position_hover_state"
topics_of_interest = [
    demo_state_topic,
    f"{mavros_ns}/state",
    f"{mavros_ns}/local_position/pose",
    f"{mavros_ns}/vision_pose/pose",
    f"{mavros_ns}/companion_process/status",
    f"{mavros_ns}/statustext/recv",
]


def read_lines(path: Path) -> list[str]:
    if not path.exists():
        return []
    return [line.rstrip() for line in path.read_text().splitlines()]


def last_nonempty(lines: list[str], limit: int) -> list[str]:
    filtered = [line for line in lines if line.strip()]
    if len(filtered) <= limit:
        return filtered
    return filtered[-limit:]


launch_processes = [line for line in read_lines(output_dir / "demo_launch_processes_start.txt") if line.strip()]
node_processes = [line for line in read_lines(output_dir / "demo_node_processes_start.txt") if line.strip()]
bridge_processes = [line for line in read_lines(output_dir / "bridge_processes_start.txt") if line.strip()]
log_files = [line for line in read_lines(output_dir / "position_hover_log_files.txt") if line.strip()]

demo_log_lines = read_lines(output_dir / "position_hover_log_tail.txt")
interesting_log_lines = [
    line for line in demo_log_lines
    if (
        "State " in line
        or "Aborting demo:" in line
        or "Requested " in line
        or "accepted" in line.lower()
        or "rejected" in line.lower()
        or "warn" in line.lower()
    )
]

if not interesting_log_lines:
    candidate_logs = sorted(Path.home().joinpath(".ros", "log").glob("python3_*.log"))
    recent_matching_logs = []
    for path in reversed(candidate_logs):
        try:
            if path.stat().st_mtime < capture_start_epoch_s - 5.0:
                continue
            text = path.read_text()
        except Exception:
            continue
        if "position_hover_demo_sequence_node" not in text:
            continue
        recent_matching_logs.append(path)
        if len(recent_matching_logs) >= 3:
            break

    for path in reversed(recent_matching_logs):
        log_files.append(str(path))
        for line in path.read_text().splitlines():
            if (
                "State " in line
                or "Aborting demo:" in line
                or "Requested " in line
                or "accepted" in line.lower()
                or "rejected" in line.lower()
                or "warn" in line.lower()
            ):
                interesting_log_lines.append(f"[fallback {path.name}] {line}")

last_abort = next((line for line in reversed(interesting_log_lines) if "Aborting demo:" in line), "")
last_state_transition = next((line for line in reversed(interesting_log_lines) if "State " in line), "")
last_request = next((line for line in reversed(interesting_log_lines) if "Requested " in line), "")

demo_state_trace = read_lines(output_dir / "demo_state_trace.txt")
statustext_trace = read_lines(output_dir / "statustext_trace.txt")
mavros_mode_trace = read_lines(output_dir / "mavros_mode_trace.txt")
mavros_connected_trace = read_lines(output_dir / "mavros_connected_trace.txt")

bag_dir = output_dir / "bag"
db_paths = sorted(bag_dir.glob("*.db3"))
topic_stats: dict[str, dict[str, float | int | None]] = {}

if db_paths:
    conn = sqlite3.connect(db_paths[0])
    cur = conn.cursor()

    def timestamps_for(topic: str) -> list[int]:
        cur.execute(
            """
            SELECT messages.timestamp
            FROM messages
            JOIN topics ON messages.topic_id = topics.id
            WHERE topics.name = ?
            ORDER BY messages.timestamp
            """,
            (topic,),
        )
        return [row[0] for row in cur.fetchall()]

    for topic in topics_of_interest:
        ts = timestamps_for(topic)
        if not ts:
            topic_stats[topic] = {"count": 0, "max_gap_s": None}
            continue
        gaps_s = [(b - a) / 1e9 for a, b in zip(ts, ts[1:])]
        topic_stats[topic] = {
            "count": len(ts),
            "max_gap_s": max(gaps_s) if gaps_s else 0.0,
        }

    conn.close()

summary_lines: list[str] = []
summary_lines.append("Position hover debug capture")
summary_lines.append("")
summary_lines.append(f"Output dir: {output_dir}")
summary_lines.append(f"Demo state topic: {demo_state_topic}")
summary_lines.append("")
summary_lines.append("Quick take")

if len(launch_processes) > 1 or len(node_processes) > 1:
    summary_lines.append(
        f"- Duplicate demo processes were present at capture start: launch={len(launch_processes)} node={len(node_processes)}"
    )
else:
    summary_lines.append(
        f"- Demo process count at capture start looked clean: launch={len(launch_processes)} node={len(node_processes)}"
    )

if last_abort:
    summary_lines.append(f"- Last abort line: {last_abort}")
elif last_state_transition:
    summary_lines.append(f"- Last state transition: {last_state_transition}")
else:
    summary_lines.append("- No fresh position-hover node log lines were captured.")

if last_request:
    summary_lines.append(f"- Last request line: {last_request}")

summary_lines.append("")
summary_lines.append("Topic stats")
for topic in topics_of_interest:
    stats = topic_stats.get(topic)
    if not stats:
        summary_lines.append(f"- {topic}: no bag data")
        continue
    count = int(stats["count"])
    max_gap_s = stats["max_gap_s"]
    if max_gap_s is None:
        summary_lines.append(f"- {topic}: count={count}, max_gap_s=n/a")
    else:
        summary_lines.append(f"- {topic}: count={count}, max_gap_s={max_gap_s:.3f}")

summary_lines.append("")
summary_lines.append("Recent demo states")
recent_states = last_nonempty(demo_state_trace, 12)
if recent_states:
    for line in recent_states:
        summary_lines.append(f"- {line}")
else:
    summary_lines.append("- No demo-state trace lines captured.")

summary_lines.append("")
summary_lines.append("Recent MAVROS mode samples")
recent_modes = last_nonempty(mavros_mode_trace, 8)
if recent_modes:
    for line in recent_modes:
        summary_lines.append(f"- {line}")
else:
    summary_lines.append("- No mode trace lines captured.")

summary_lines.append("")
summary_lines.append("Recent MAVROS connected samples")
recent_connected = last_nonempty(mavros_connected_trace, 8)
if recent_connected:
    for line in recent_connected:
        summary_lines.append(f"- {line}")
else:
    summary_lines.append("- No connected trace lines captured.")

summary_lines.append("")
summary_lines.append("Recent statustext")
recent_statustext = last_nonempty(statustext_trace, 10)
if recent_statustext:
    for line in recent_statustext:
        summary_lines.append(f"- {line}")
else:
    summary_lines.append("- No statustext messages captured.")

summary_lines.append("")
summary_lines.append("Recent node log lines")
recent_logs = last_nonempty(interesting_log_lines, 12)
if recent_logs:
    for line in recent_logs:
        summary_lines.append(f"- {line}")
else:
    summary_lines.append("- No fresh filtered node-log lines captured.")

summary_lines.append("")
summary_lines.append("Artifacts")
summary_lines.append("- bag/")
summary_lines.append("- demo_state_trace.txt")
summary_lines.append("- mavros_mode_trace.txt")
summary_lines.append("- mavros_connected_trace.txt")
summary_lines.append("- statustext_trace.txt")
summary_lines.append("- position_hover_log_tail.txt")
summary_lines.append("- demo_launch_processes_start.txt")
summary_lines.append("- demo_node_processes_start.txt")
summary_lines.append("- bridge_processes_start.txt")
if log_files:
    summary_lines.append("- position_hover_log_files.txt")

summary_text = "\n".join(summary_lines) + "\n"
(output_dir / "summary.txt").write_text(summary_text)
print(summary_text, end="")
PY
}

handle_exit() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_background_jobs
  if [[ -n "${OUTPUT_DIR}" && -d "${OUTPUT_DIR}" ]]; then
    generate_summary
  fi
  exit "${exit_code}"
}

start_field_trace() {
  local topic="$1"
  local field="$2"
  local outfile="$3"

  (
    set +e
    timeout --signal=INT --kill-after=5s "${DURATION_S}s" \
      ros2 topic echo "${topic}" --field "${field}" 2>/dev/null \
      | grep --line-buffered -v '^---$' \
      | grep --line-buffered -v '^WARNING: topic ' \
      | while IFS= read -r line; do
          if [[ -z "${line}" ]]; then
            continue
          fi
          printf '%s %s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "${line}"
        done \
      > "${outfile}"
  ) &
  JOB_PIDS+=("$!")
}

have_tailed_log_key() {
  local key="$1"
  local existing
  for existing in "${TAILED_LOG_KEYS[@]:-}"; do
    if [[ "${existing}" == "${key}" ]]; then
      return 0
    fi
  done
  return 1
}

register_demo_logs() {
  local pid
  local log_file
  local key

  mapfile -t DEMO_PIDS < <(pgrep -f "position_hover_demo_sequence_node" || true)
  for pid in "${DEMO_PIDS[@]:-}"; do
    log_file="$(find "${HOME}/.ros/log" -maxdepth 1 -type f -name "python3_${pid}_*.log" | sort | tail -n 1 || true)"
    if [[ -z "${log_file}" ]]; then
      continue
    fi
    key="${pid}:${log_file}"
    if have_tailed_log_key "${key}"; then
      continue
    fi
    TAILED_LOG_KEYS+=("${key}")
    DEMO_LOG_FILES+=("${key}")
    printf '%s %s\n' "${pid}" "${log_file}" >> "${OUTPUT_DIR}/position_hover_log_files.txt"
    (
      set +e
      stdbuf -oL tail -n 0 -F "${log_file}" 2>/dev/null \
        | sed -u "s#^#[pid ${pid}] #g" \
        >> "${OUTPUT_DIR}/position_hover_log_tail.txt"
    ) &
    JOB_PIDS+=("$!")
  done
}

monitor_demo_until_terminal_state() {
  local deadline_s=$(( CAPTURE_START_EPOCH_S + DURATION_S ))
  local seen_active=0
  local last_line=""
  local last_state=""

  while (( "$(date +%s)" < deadline_s )); do
    register_demo_logs

    if ! kill -0 "${BAG_PID}" >/dev/null 2>&1; then
      return 0
    fi

    if [[ -s "${OUTPUT_DIR}/demo_state_trace.txt" ]]; then
      last_line="$(tail -n 1 "${OUTPUT_DIR}/demo_state_trace.txt" 2>/dev/null || true)"
      if [[ -n "${last_line}" ]]; then
        last_state="${last_line##* }"
        if [[ "${last_state}" != "IDLE" ]]; then
          seen_active=1
        fi
        if [[ "${seen_active}" -eq 1 && ( "${last_state}" == "ABORT" || "${last_state}" == "COMPLETE" ) ]]; then
          echo "Detected terminal demo state: ${last_state}"
          return 0
        fi
      fi
    fi

    sleep 0.25
  done

  echo "Capture duration reached without a terminal demo state."
  return 0
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION_S="$2"
      shift 2
      ;;
    --drone-id)
      DRONE_ID="$2"
      shift 2
      ;;
    --mavros-namespace)
      MAVROS_NAMESPACE="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --extra-topic)
      EXTRA_TOPICS+=("$2")
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if ! [[ "${DURATION_S}" =~ ^[0-9]+$ ]] || [[ "${DURATION_S}" -le 0 ]]; then
  echo "--duration must be a positive integer number of seconds." >&2
  exit 1
fi

MAVROS_NAMESPACE="$(normalize_ns "${MAVROS_NAMESPACE}")"

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/temp_outputs/position_hover_capture_${STAMP}"
fi

mkdir -p "${OUTPUT_DIR}"

trap handle_exit EXIT INT TERM

source_ros_env

DEMO_STATE_TOPIC="/cdrone/${DRONE_ID}/demo/position_hover_state"
DEMO_START_SERVICE="/cdrone/${DRONE_ID}/demo/position_hover_start"
DEMO_ABORT_SERVICE="/cdrone/${DRONE_ID}/demo/position_hover_abort"

TOPICS=(
  "/rosout"
  "${DEMO_STATE_TOPIC}"
  "${MAVROS_NAMESPACE}/state"
  "${MAVROS_NAMESPACE}/local_position/pose"
  "${MAVROS_NAMESPACE}/vision_pose/pose"
  "${MAVROS_NAMESPACE}/companion_process/status"
  "${MAVROS_NAMESPACE}/statustext/recv"
  "${MAVROS_NAMESPACE}/status_event"
  "${MAVROS_NAMESPACE}/extended_state"
  "${MAVROS_NAMESPACE}/param/event"
)

for topic in "${EXTRA_TOPICS[@]}"; do
  TOPICS+=("${topic}")
done

{
  echo "stamp=${STAMP}"
  echo "duration_s=${DURATION_S}"
  echo "drone_id=${DRONE_ID}"
  echo "mavros_namespace=${MAVROS_NAMESPACE}"
  echo "output_dir=${OUTPUT_DIR}"
  echo "start_service=${DEMO_START_SERVICE}"
  echo "abort_service=${DEMO_ABORT_SERVICE}"
  echo "topics="
  printf '  %s\n' "${TOPICS[@]}"
} > "${OUTPUT_DIR}/capture_config.txt"

printf '%s\n' "${TOPICS[@]}" > "${OUTPUT_DIR}/topics_requested.txt"

echo "Capturing position hover debug for up to ${DURATION_S}s"
echo "Output: ${OUTPUT_DIR}"
echo "Observer mode only: launch and trigger the demo separately."
echo ""

ros2 node list | sort > "${OUTPUT_DIR}/ros_nodes_start.txt" || true
ros2 topic list | sort > "${OUTPUT_DIR}/ros_topics_start.txt" || true
ros2 service list | sort > "${OUTPUT_DIR}/ros_services_start.txt" || true
ros2 topic info -v "${DEMO_STATE_TOPIC}" > "${OUTPUT_DIR}/demo_state_topic_info_start.txt" || true

pgrep -af "ros2 launch drone_bringup position_hover_demo.launch.py" \
  > "${OUTPUT_DIR}/demo_launch_processes_start.txt" || true
pgrep -af "position_hover_demo_sequence_node" \
  > "${OUTPUT_DIR}/demo_node_processes_start.txt" || true
pgrep -af "external_pose_adapter_node|external_pose_bridge_node|vrpn_mocap_client_node|mavros_node" \
  > "${OUTPUT_DIR}/bridge_processes_start.txt" || true

: > "${OUTPUT_DIR}/position_hover_log_files.txt"
: > "${OUTPUT_DIR}/position_hover_log_tail.txt"
register_demo_logs

start_field_trace "${DEMO_STATE_TOPIC}" "data" "${OUTPUT_DIR}/demo_state_trace.txt"
start_field_trace "${MAVROS_NAMESPACE}/statustext/recv" "text" "${OUTPUT_DIR}/statustext_trace.txt"
start_field_trace "${MAVROS_NAMESPACE}/state" "mode" "${OUTPUT_DIR}/mavros_mode_trace.txt"
start_field_trace "${MAVROS_NAMESPACE}/state" "connected" "${OUTPUT_DIR}/mavros_connected_trace.txt"

(
  set +e
  timeout --signal=INT --kill-after=20s "${DURATION_S}s" \
    ros2 bag record -o "${OUTPUT_DIR}/bag" "${TOPICS[@]}" \
    > "${OUTPUT_DIR}/bag_record.log" 2>&1
) &
BAG_PID="$!"
JOB_PIDS+=("${BAG_PID}")

monitor_demo_until_terminal_state || true
stop_background_jobs
generate_summary

echo ""
echo "Saved position hover debug capture to ${OUTPUT_DIR}"
echo "Summary: ${OUTPUT_DIR}/summary.txt"

trap - EXIT INT TERM
