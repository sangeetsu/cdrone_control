#!/bin/bash
# Quick rebuild and test for keyboard teleop
# (Auto-sources workspace from ~/.bashrc)

set -e

cd /home/jetson/ros_ws/cdrone_control/ros2

echo "=========================================="
echo "Building keyboard teleop..."
echo "=========================================="
colcon build --packages-select drone_control_pkg

echo ""
echo "=========================================="
echo "✅ Build Complete!"
echo "=========================================="
echo ""
echo "Verifying keyboard_teleop_node is available..."
if ros2 pkg executables drone_control_pkg | grep -q "keyboard_teleop_node"; then
    echo "   ✓ keyboard_teleop_node found"
else
    echo "   ✗ keyboard_teleop_node NOT found"
    exit 1
fi

echo ""
echo "=========================================="
echo "Ready to Run!"
echo "=========================================="
echo ""
echo "Open 3 terminals and run:"
echo ""
echo "Terminal 1 (MAVROS):"
echo "  cd /home/jetson/ros_ws/cdrone_control/ros2"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 launch drone_bringup drone.launch.py"
echo ""
echo "Terminal 2 (Velocity Node):"
echo "  cd /home/jetson/ros_ws/cdrone_control/ros2"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 run drone_control_pkg mavros_velocity_node \\"
echo "    --ros-args -p require_guided_mode:=false"
echo ""
echo "Terminal 3 (Keyboard Teleop):"
echo "  cd /home/jetson/ros_ws/cdrone_control/ros2"
echo "  source /opt/ros/humble/setup.bash"
echo "  source install/setup.bash"
echo "  ros2 run drone_control_pkg keyboard_teleop_node"
echo ""
echo "Then in keyboard terminal:"
echo "  1 - ARM"
echo "  2 - STABILIZED mode"
echo "  W/S/A/D/R/F/Q/E - Control"
echo "  SPACE - Stop"
echo "  4 - DISARM"
echo ""
