#!/bin/bash

set -euo pipefail

check_pkg() {
  local pkg="$1"
  if dpkg -s "${pkg}" >/dev/null 2>&1; then
    echo "  ✓ ${pkg}"
  else
    echo "  ✗ ${pkg}"
  fi
}

echo "Host VIO status"
echo "==============="

echo "ROS install:"
if [[ -d /opt/ros/humble ]]; then
  echo "  ✓ /opt/ros/humble"
else
  echo "  ✗ /opt/ros/humble"
fi

echo ""
echo "Core packages:"
check_pkg ros-humble-ros-base
check_pkg ros-humble-mavros
check_pkg ros-humble-mavros-extras
check_pkg ros-humble-realsense2-camera
check_pkg ros-humble-cv-bridge
check_pkg ros-humble-image-transport
check_pkg ros-humble-imu-filter-madgwick

echo ""
echo "RealSense visibility:"
if lsusb | grep -Eiq '8086:0b5c|RealSense'; then
  echo "  ✓ RealSense USB device visible"
else
  echo "  ✗ RealSense USB device visible"
fi

if [[ -f /etc/udev/rules.d/99-realsense-libusb.rules ]]; then
  echo "  ✓ RealSense udev rules installed"
else
  echo "  ✗ RealSense udev rules installed"
fi

echo ""
echo "Privilege checks:"
if sudo -n true 2>/dev/null; then
  echo "  ✓ passwordless sudo"
else
  echo "  ✗ passwordless sudo"
fi

if docker ps >/dev/null 2>&1; then
  echo "  ✓ docker socket access"
else
  echo "  ✗ docker socket access"
fi
