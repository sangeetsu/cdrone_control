#!/usr/bin/env bash
set -euo pipefail

echo "== lsusb =="
lsusb | grep -i -E 'intel|realsense' || true

echo
echo "== rs-enumerate-devices =="
rs-enumerate-devices || true

echo
echo "== matching /dev/bus/usb entries =="
while read -r bus_num dev_num; do
  [[ -z "${bus_num}" || -z "${dev_num}" ]] && continue
  device_path="/dev/bus/usb/${bus_num}/${dev_num}"
  [[ -e "${device_path}" ]] && ls -l "${device_path}"
done < <(
  lsusb | grep -i -E 'intel|realsense' | awk '{gsub(":", "", $4); print $2, $4}'
) || true
