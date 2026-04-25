#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RULES_SRC="${SCRIPT_DIR}/blink1/99-blink1.rules"
RULES_DST="/etc/udev/rules.d/99-blink1.rules"

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
echo "If the blink(1) is already plugged in, unplug and replug it once before retrying."
