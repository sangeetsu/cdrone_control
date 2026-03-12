#!/bin/bash
# Quick test to verify mavlink-router and MAVROS UDP connection

echo "=========================================="
echo "Testing mavlink-router + MAVROS Setup"
echo "=========================================="
echo ""

# Check mavlink-router service
echo "1. Checking mavlink-router process..."
if pgrep -x "mavlink-routerd" >/dev/null; then
    echo "   ✅ mavlink-routerd is RUNNING"
    ps aux | grep "mavlink-routerd" | grep -v grep
else
    echo "   ❌ mavlink-routerd is NOT running"
    echo "   Check if it's configured to start automatically"
    echo "   Or run manually: /home/jetson/.local/bin/start_mavlink_router.sh"
    exit 1
fi

echo ""
echo "2. Checking UDP endpoints (listening ports)..."
if command -v ss >/dev/null 2>&1; then
    echo "   UDP ports 14540 (MAVROS) and 14550 (QGC):"
    ss -ulpn | grep -E "14540|14550" | grep -v grep || echo "   (Endpoints will appear when clients connect)"
else
    echo "   (ss command not available, skipping)"
fi
echo ""

# Check USB connection
echo "3. Checking FC USB connection..."
if ls /dev/serial/by-id/usb-ARK* >/dev/null 2>&1; then
    echo "   ✅ FC USB connected:"
    ls -l /dev/serial/by-id/usb-ARK*
else
    echo "   ❌ FC USB not found"
    echo "   Check USB cable connection"
    exit 1
fi

echo ""
echo "4. Checking MAVROS configuration..."
CONFIG_FILE="/home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/config/apm_params.yaml"
if grep -q "udp://:14540@127.0.0.1:14550" "$CONFIG_FILE"; then
    echo "   ✅ MAVROS configured for UDP (mavlink-router)"
    grep "fcu_url" "$CONFIG_FILE"
else
    echo "   ⚠️  MAVROS not configured for UDP"
    echo "   Current config:"
    grep "fcu_url" "$CONFIG_FILE"
    echo ""
    echo "   Should be: fcu_url: \"udp://:14540@127.0.0.1:14550\""
fi

echo ""
echo "=========================================="
echo "✅ Pre-flight checks complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Terminal 1: ros2 launch drone_bringup drone.launch.py"
echo "  2. Terminal 2: ros2 run drone_control_pkg mavros_velocity_node"
echo "  3. Terminal 3: ros2 run drone_control_pkg keyboard_teleop_node"
echo ""
echo "QGroundControl can run simultaneously on UDP 14550!"
echo ""
