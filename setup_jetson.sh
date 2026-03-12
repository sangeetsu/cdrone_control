#!/bin/bash
# Setup script for cdrone_control on Jetson Orin Nano + ARK PAB Carrier + PX4
# Run this once to install all dependencies and build the workspace

set -e  # Exit on error

echo "=========================================="
echo "cdrone_control Setup for Jetson + PX4"
echo "=========================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 1. Install ROS2 MAVROS packages
echo "Step 1/6: Installing MAVROS..."
sudo apt update
sudo apt install -y \
  ros-humble-mavros \
  ros-humble-mavros-extras \
  ros-humble-geographic-msgs \
  python3-pip \
  python3-opencv

echo "   ✓ MAVROS installed"
echo ""

# 2. Install GeographicLib datasets (required by MAVROS)
echo "Step 2/6: Installing GeographicLib datasets..."
if [ -d "/usr/share/GeographicLib" ]; then
    echo "   ✓ GeographicLib datasets already installed"
else
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh
    rm install_geographiclib_datasets.sh
    echo "   ✓ GeographicLib datasets installed"
fi
echo ""

# 3. Install Python dependencies
echo "Step 3/6: Installing Python dependencies..."
if [ -f "requirements-jetson.txt" ]; then
    pip3 install -r requirements-jetson.txt
    echo "   ✓ Python dependencies installed"
else
    echo "   ⚠ requirements-jetson.txt not found, skipping"
fi
echo ""

# 4. Source ROS2
echo "Step 4/6: Sourcing ROS2 Humble..."
source /opt/ros/humble/setup.bash
echo "   ✓ ROS2 sourced"
echo ""

# 5. Build workspace
echo "Step 5/6: Building ROS2 workspace..."
cd ros2
rm -rf build install log
colcon build --symlink-install --parallel-workers 1 --executor sequential
echo "   ✓ Workspace built"
echo ""

# 6. Verification
echo "Step 6/6: Verifying installation..."
source install/setup.bash

# Check if packages are available
ERRORS=0
for pkg in drone_bringup drone_control_pkg drone_behavior_pkg drone_vision_pkg drone_light_pkg drone_msgs ros2_poselib; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
        echo "   ✓ ${pkg}"
    else
        echo "   ✗ ${pkg} - NOT FOUND"
        ERRORS=$((ERRORS + 1))
    fi
done

echo ""
if [ $ERRORS -eq 0 ]; then
    echo "=========================================="
    echo "✅ Setup Complete!"
    echo "=========================================="
    echo ""
    echo "Next steps:"
    echo "1. Ensure PX4 flight controller is connected to /dev/ttyACM0"
    echo "2. Source the workspace:"
    echo "   cd $SCRIPT_DIR/ros2"
    echo "   source install/setup.bash"
    echo ""
    echo "3. Launch MAVROS only (test connection):"
    echo "   ros2 launch drone_bringup drone.launch.py"
    echo ""
    echo "4. Or launch full autonomy stack:"
    echo "   ros2 launch drone_bringup autonomy_stack.launch.py"
    echo ""
else
    echo "=========================================="
    echo "⚠ Setup completed with $ERRORS errors"
    echo "=========================================="
    echo "Please review the errors above and try building again."
fi
