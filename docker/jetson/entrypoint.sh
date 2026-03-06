#!/bin/bash

set -e

# source the ros2 workspace
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/local_setup.bash

# wait for mavros node
sleep 5

SOURCE_MODE=${SOURCE_MODE:-csi}
SCENARIO=${SCENARIO:-intercept_illuminate_v1}

ros2 launch drone_bringup autonomy_stack.launch.py source_mode:=${SOURCE_MODE} scenario:=${SCENARIO}
