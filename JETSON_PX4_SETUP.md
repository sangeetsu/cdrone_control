# cdrone_control + PX4 Setup Guide (MAVROS-based)

**Hardware:** Jetson Orin Nano + ARK Jetson PAB Carrier + ARKV6X + PX4 1.16.0  
**Communication:** MAVROS (MAVLink-ROS2 bridge)  
**Advantage:** Works without PX4 external modes - uses standard MAVLink

---

## 🔄 Differences from ros2_px4_teleop_example

| Feature | teleop_example | cdrone_control (MAVROS) |
|---------|----------------|-------------------------|
| **Communication** | uXRCE-DDS (direct PX4 ↔ ROS2) | MAVLink (via mavlink-router) |
| **PX4 Requirement** | External modes support | Standard MAVLink (any PX4) |
| **Hardware Link** | TELEM2 UART (`/dev/ttyTHS1`) | UDP via mavlink-router |
| **Custom Modes** | Registers "Teleoperation" mode | Uses standard GUIDED/OFFBOARD |
| **Setup Complexity** | Higher (uXRCE config) | Lower (just MAVROS) |
| **Message Versions** | Must match PX4 version exactly | MAVLink is more stable |
| **QGC Compatible** | No (conflicts) | Yes (both via mavlink-router) |

---

## ✅ What We Learned from teleop_example That Still Applies

### 1. **Serial Device Paths**
- ✓ `/dev/ttyACM0` - Flight controller USB (921600 baud)
- ✓ `/dev/ttyTHS1` - TELEM2 UART (not needed for MAVROS)

### 2. **Branch Alignment** 
- ❌ **NOT needed for MAVROS!** 
- MAVROS uses MAVLink protocol, which is more stable across versions
- No need to match exact PX4 firmware versions

### 3. **Multiple Agent Conflicts**
- ✓ Still important: Only run ONE connection to each serial port
- Don't run both QGC and MAVROS on same port simultaneously
- Stop mavlink-router if running MAVROS on `/dev/ttyACM0`

### 4. **PX4 Configuration**
- ❌ **uXRCE client NOT needed** (that was for teleop example only)
- ✓ Keep MAVLink enabled on USB (should be default)
- ✓ QGroundControl calibration/params still useful

---

## 🚀 Quick Start: cdrone_control

### Setup (One-Time)

```bash
cd /home/jetson/ros_ws/cdrone_control
chmod +x setup_jetson.sh
./setup_jetson.sh
```

This will:
- Install MAVROS and dependencies
- Install GeographicLib datasets
- Install Python packages
- Build the ROS2 workspace

---

### Running MAVROS (Every Time)

**Note:** ROS2 and workspace are auto-sourced in `~/.bashrc`. Just open new terminals!

**Great News:** mavlink-router is already running as a service! 
- QGC and MAVROS can run **simultaneously** via UDP
- No conflicts, no serial port issues

**Terminal 1: Launch MAVROS**
```bash
# Launch MAVROS only (test connection)
ros2 launch drone_bringup drone.launch.py
```

**Success check:**
```
[INFO] [mavros_node]: FCU: PX4 Autopilot vX.X.X
[INFO] [mavros_node]: FCU: ARKV6X
[INFO] [mavros_node]: Connected to FCU
```

**Terminal 3: Verify MAVROS topics**
```bash
ros2 topic list | grep mavros
```

Should see many `/mavros/...` topics:
```
/mavros/state
/mavros/imu/data
/mavros/local_position/pose
/mavros/setpoint_velocity/cmd_vel_unstamped
...
```

---

### Full Autonomy Stack

**Once MAVROS connection is verified:**

```bash
# Launch full stack (MAVROS + control + vision + behavior)
ros2 launch drone_bringup autonomy_stack.launch.py
```

---

## 📋 Comparison: What Works Now vs Before

### ✅ Confirmed Working (from teleop testing):

1. **Serial Communication**
   - ✓ `/dev/ttyACM0` exists and accessible
   - ✓ 921600 baud rate works
   - ✓ User in `dialout` group

2. **PX4 Firmware**
   - ✓ PX4 1.16.0 running
   - ✓ ARKV6X hardware detected
   - ✓ MAVLink communication working (QGC connected before)

3. **ROS2 Environment**
   - ✓ ROS2 Humble installed
   - ✓ Workspaces build successfully
   - ✓ Topic communication works

### ⚠️ What's Different:

1. **No uXRCE-DDS needed**
   - Don't need to run MicroXRCEAgent
   - Don't need to configure `UXRCE_DDS_CFG`
   - Don't need `/dev/ttyTHS1` UART

2. **No external modes needed**
   - Don't need `COM_EXT_MODES` parameter
   - MAVROS uses standard MAVLink modes (GUIDED, OFFBOARD, etc.)
   - Works with your current PX4 firmware

3. **Simpler connection**
   - Just USB serial (`/dev/ttyACM0`)
   - No dual-link setup required
   - MAVROS handles all MAVLink communication

---

## 🔧 Configuration Files to Review

### PX4 Parameters (set in QGC if needed):

```
# MAVLink on USB (should be default)
MAV_0_CONFIG = TELEM 1    # USB serial
MAV_0_MODE = Normal
MAV_0_RATE = 1200         # Data rate

# Offboard control (for velocity commands)
COM_OF_LOSS_T = 0.5       # Offboard loss timeout (seconds)
COM_OBL_RC_ACT = 0        # Action when RC lost in offboard

# For indoor flight (no GPS)
COM_ARM_WO_GPS = 1
EKF2_GPS_CHECK = 0
CBRK_GPSFAIL = 240024
```

### MAVROS Config Files:

Already configured in cdrone_control:
- `ros2/src/drone_bringup/config/apm_params.yaml` - FCU URL: `/dev/ttyACM0:921600`
- `ros2/src/drone_bringup/config/apm_config.yaml` - MAVROS plugin settings
- `ros2/src/drone_bringup/config/apm_pluginlists.yaml` - Enabled plugins

⚠️ **Note:** Files say "APM" (ArduPilot) but MAVROS works with both ArduPilot and PX4!

---

## 🎮 Control via MAVROS

### Arming via ROS2 (instead of QGC):

```bash
# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

### Set Mode:

```bash
# Set to OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Set to STABILIZED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'STABILIZED'}"
```

### Send Velocity Commands:

```bash
# Forward at 0.5 m/s
ros2 topic pub /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 🆚 MAVROS vs QGroundControl

You can use **BOTH** or **EITHER**:

### Option 1: MAVROS Only (No QGC)
- ✓ Arm/disarm via ROS2 services
- ✓ Mode changes via ROS2 services  
- ✓ Send velocity commands via ROS2 topics
- ✓ Monitor state via ROS2 topics
- ❌ No GUI for parameters/calibration

### Option 2: QGC + MAVROS Together
- Need to share the serial port using `mavlink-router`:

**Terminal 1: MAVLink Router**
```bash
sudo /home/jetson/.local/bin/mavlink-routerd \
  -e 192.168.0.215:14550 \
  -e 127.0.0.1:14540 \
  /dev/ttyACM0:921600
```

**Terminal 2: MAVROS (connect to localhost)**

Edit `apm_params.yaml`:
```yaml
mavros_node:
  ros__parameters:
    fcu_url: "udp://:14540@127.0.0.1:14557"
```

Then launch MAVROS normally.

Now QGC (on 192.168.0.215) and MAVROS (localhost) can both communicate with PX4!

---

## 🐛 Troubleshooting

### Issue: MAVROS can't connect to FCU

**Check 1: Serial port accessible**
```bash
ls -l /dev/ttyACM0
# Should exist with dialout group
```

**Check 2: No conflicts**
```bash
sudo lsof /dev/ttyACM0
# Should show nothing or only MAVROS
```

**Check 3: Kill conflicting processes**
```bash
sudo pkill mavlink-routerd
sudo pkill qgroundcontrol
```

**Check 4: MAVROS logs**
```bash
ros2 launch drone_bringup drone.launch.py log_level:=debug
```

Look for:
- `[ INFO] [mavros_node]: FCU: PX4 Autopilot` ✓
- `[ERROR] [mavros_node]: serial0: device open failed` ✗

---

### Issue: No `/mavros/...` topics

**Cause:** MAVROS node not running

**Fix:** Launch drone.launch.py (see Terminal 2 above)

---

### Issue: Can't arm via MAVROS

**Common pre-arm checks that apply:**
- ✓ Accelerometer calibrated
- ✓ Level horizon set
- ✓ Indoor parameters configured (if no GPS)
- ✓ Battery connected and voltage OK

**Check arm status:**
```bash
ros2 topic echo /mavros/state
# Look for: armed: false/true
```

**Try forcing arm (props off!):**
```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

---

## 📊 What's in cdrone_control

Your repo includes a full autonomy stack beyond just MAVROS:

1. **drone_bringup** - Launch files for MAVROS and full stack
2. **drone_control_pkg** - Velocity control, setup node, MAVROS integration
3. **drone_behavior_pkg** - State machine for target engagement
4. **drone_vision_pkg** - Stereo tracking with TensorRT
5. **drone_light_pkg** - Spotlight control via GPIO
6. **drone_msgs** - Custom message types

This is **much more complete** than the simple teleop example!

---

## 🎯 Summary: Why This is Better

### Teleop Example Issues:
- ❌ Requires external modes (your PX4 doesn't have it)
- ❌ Complex dual-link setup (USB + UART)
- ❌ Exact version matching required
- ❌ Only provides keyboard teleop

### cdrone_control Advantages:
- ✅ Works with any PX4 firmware (just needs MAVLink)
- ✅ Single USB connection
- ✅ More flexible (MAVROS is widely supported)
- ✅ Full autonomy stack included
- ✅ Proven solution (MAVLink is mature)

---

## 🚀 Next Steps

1. **Run setup script:**
   ```bash
   cd /home/jetson/ros_ws/cdrone_control
   ./setup_jetson.sh
   ```

2. **Test MAVROS connection:**
   ```bash
   ros2 launch drone_bringup drone.launch.py
   ```

3. **Verify topics:**
   ```bash
   ros2 topic list | grep mavros
   ```

4. **Test arming (props off!):**
   ```bash
   ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
   ```

5. **Move to full autonomy when ready!**

---

**You're on a much better path now. MAVROS doesn't care about external modes!** 🎉
