#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/realsense/99-realsense-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-realsense-libusb.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script with sudo:"
  echo "  sudo ${0}"
  exit 1
fi

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "Missing source rules file: ${RULES_SRC}" >&2
  exit 1
fi

install -m 0644 "${RULES_SRC}" "${RULES_DST}"
udevadm control --reload-rules
udevadm trigger

echo "Installed ${RULES_DST}."
echo "If the D455 is already plugged in, unplug and replug it once before retrying."
