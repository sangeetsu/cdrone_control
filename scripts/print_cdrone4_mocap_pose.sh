#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

TOPIC="/cdrone/cdrone4/vrpn_mocap/RigidBody4/pose"
WATCH=0
TIMEOUT_S=5

usage() {
  cat <<'EOF'
Usage:
  scripts/print_cdrone4_mocap_pose.sh [--watch] [--topic TOPIC] [--timeout SEC]

Print the current cdrone4 OptiTrack/VRPN mocap coordinates.

Options:
  --watch        Continuously print mocap samples.
  --topic TOPIC  Override the pose topic.
  --timeout SEC  Seconds to wait for one sample. Default: 5.
  -h, --help     Show this help.
EOF
}

fail() {
  printf '[mocap_pose] ERROR: %s\n' "$*" >&2
  exit 1
}

source_ros_env() {
  set +u
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
  fi
  if [[ -f "${REPO_ROOT}/ros2/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${REPO_ROOT}/ros2/install/setup.bash"
  fi
  set -u

  command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available after sourcing ROS/workspace setup."
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --watch)
      WATCH=1
      shift
      ;;
    --topic)
      [[ $# -ge 2 ]] || fail "--topic requires a value."
      TOPIC="$2"
      shift 2
      ;;
    --timeout)
      [[ $# -ge 2 ]] || fail "--timeout requires a value."
      TIMEOUT_S="$2"
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

source_ros_env

print_once() {
  local sample
  sample="$(timeout "${TIMEOUT_S}s" ros2 topic echo --no-daemon --spin-time 2 --once "${TOPIC}" 2>/dev/null || true)"
  [[ -n "${sample}" ]] || fail "No mocap sample received from ${TOPIC}"

  python3 -c '
import re
import sys

text = sys.stdin.read()

def get(path):
    pattern = r"%s:\s*\n\s*x:\s*([-+0-9.eE]+)\s*\n\s*y:\s*([-+0-9.eE]+)\s*\n\s*z:\s*([-+0-9.eE]+)" % path
    match = re.search(pattern, text)
    if not match:
        return None
    return tuple(float(v) for v in match.groups())

stamp_match = re.search(r"stamp:\s*\n\s*sec:\s*([0-9]+)\s*\n\s*nanosec:\s*([0-9]+)", text)
pos = get("position")
orient = get("orientation")
if pos is None:
    print(text.strip())
    sys.exit(0)

stamp = ""
if stamp_match:
    stamp = f" t={stamp_match.group(1)}.{int(stamp_match.group(2)):09d}"

line = f"{stamp} x={pos[0]: .3f} m  y={pos[1]: .3f} m  z={pos[2]: .3f} m"
if orient is not None:
    line += f"  qx={orient[0]: .4f} qy={orient[1]: .4f} qz={orient[2]: .4f}"
print(line.strip())
' <<<"${sample}"
}

if [[ "${WATCH}" -eq 1 ]]; then
  while true; do
    print_once
    sleep 0.25
  done
fi

print_once
