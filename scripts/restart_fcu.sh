#!/bin/bash
# restart_fcu.sh
# Reboots the PX4 flight controller (FCU) via MAVROS using
# MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (command=246, param1=1.0).

set -e

source /opt/ros/humble/setup.bash
source /home/jetson/cdrone_control/ros2/install/setup.bash

echo "[restart_fcu] Sending reboot command to FCU via /mavros/cmd/command ..."

ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong \
  "{broadcast: false, command: 246, confirmation: 0, param1: 1.0, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"

echo "[restart_fcu] Reboot command sent. FCU should restart momentarily."
