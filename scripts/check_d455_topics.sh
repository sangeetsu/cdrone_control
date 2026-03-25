#!/bin/bash

set -euo pipefail

CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-camera}"
CAMERA_NAME="${CAMERA_NAME:-d455}"
TOPIC_ROOT="/${CAMERA_NAMESPACE}/${CAMERA_NAME}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not installed or not on PATH."
  exit 1
fi

TOPICS=(
  "${TOPIC_ROOT}/infra1/image_rect_raw"
  "${TOPIC_ROOT}/infra2/image_rect_raw"
  "${TOPIC_ROOT}/gyro/sample"
  "${TOPIC_ROOT}/accel/sample"
  "${TOPIC_ROOT}/imu"
)

echo "Checking D455 topics under ${TOPIC_ROOT}"

for topic in "${TOPICS[@]}"; do
  if ros2 topic list | grep -qx "${topic}"; then
    echo "  ✓ ${topic}"
  else
    echo "  ✗ ${topic}"
  fi
done

echo ""
echo "Sampling rates (about 5 seconds each when available):"

for topic in "${TOPICS[@]}"; do
  if ! ros2 topic list | grep -qx "${topic}"; then
    continue
  fi

  echo ""
  echo "[${topic}]"
  timeout 6 ros2 topic hz "${topic}" 2>/dev/null | tail -n 3 || true
done
