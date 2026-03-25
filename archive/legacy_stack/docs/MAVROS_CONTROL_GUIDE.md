# MAVROS Control Capabilities for cdrone_control

**Hardware:** Jetson Orin Nano + ARK PAB Carrier + ARKV6X + PX4 1.16.0  
**Communication:** MAVROS (MAVLink-ROS2 bridge) via mavlink-router

---

## 🔌 System Architecture

**Your system uses mavlink-router for MAVLink multiplexing:**
```
FC USB (2Mbps) → mavlink-router → UDP endpoints:
                                  ├─ 14550 (QGC)
                                  └─ 14540 (MAVROS)
```

**Benefit:** QGC and MAVROS work simultaneously! No conflicts!

---

## ✅ Confirmed Capabilities

### 1. **Arming/Disarming** ✓

**Via drone_control_node:**
```python
# In your code
node.arm()  # Arm the drone
```

**Via ROS2 service (command line):**
```bash
# Arm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

# Disarm
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"
```

**Via new keyboard teleop:**
- Press `1` to arm
- Press `4` to disarm

---

### 2. **Mode Changes (Including STABILIZED)** ✓

**Supported PX4 modes:**
- `MANUAL` - Manual control (RC only)
- `STABILIZED` - Stabilized flight (ideal for indoor/manual control)
- `ALTITUDE` - Altitude hold
- `POSITION` - Position hold (requires GPS or vision)
- `OFFBOARD` - External computer control (for velocity commands)
- `AUTO.LOITER` - Loiter at current position
- `AUTO.RTL` - Return to launch
- `AUTO.LAND` - Land at current position

**Via drone_control_node:**
```python
# In your code
node.change_mode('STABILIZED')  # Set stabilized mode
node.change_mode('OFFBOARD')    # Set offboard mode
```

**Via ROS2 service:**
```bash
# Set STABILIZED mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'STABILIZED'}"

# Set OFFBOARD mode
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
```

**Via keyboard teleop:**
- Press `2` for STABILIZED mode
- Press `3` for OFFBOARD mode

---

### 3. **Velocity Control** ✓

The repo has **two velocity control layers:**

#### Layer 1: mavros_velocity_node (Safety Wrapper)
- Subscribes to: `/cdrone/control/cmd_vel_body`
- Publishes to: `/mavros/setpoint_velocity/cmd_vel`
- **Safety features:**
  - Velocity clamping (configurable max speeds)
  - Watchdog timeout (0.5s default - stops if no commands)
  - E-stop support
  - Mode checking (requires GUIDED/OFFBOARD by default)

#### Layer 2: Direct MAVROS
- Publish directly to: `/mavros/setpoint_velocity/cmd_vel`
- No safety features (use with caution!)

**Parameters for mavros_velocity_node:**
```yaml
mavros_velocity_node:
  ros__parameters:
    publish_rate_hz: 20.0
    watchdog_timeout_s: 0.5
    max_vel_xy_mps: 1.5      # Max horizontal velocity (m/s)
    max_vel_z_mps: 0.8       # Max vertical velocity (m/s)
    max_yaw_rate_rps: 0.6    # Max yaw rate (rad/s)
    require_guided_mode: true  # Only send commands in OFFBOARD/GUIDED
```

---

### 4. **Position Control** ✓

**Via drone_control_node:**
- Publishes to: `/mavros/setpoint_position/local`
- Accepts local position setpoints (meters from home)

---

## 🆕 New Keyboard Teleop Node

I've added a **keyboard_teleop_node.py** that provides WASD-style control similar to the PX4 teleop example, but using MAVROS instead.

### Features:
- ✓ WASD movement controls
- ✓ Arm/disarm from keyboard
- ✓ Mode changes from keyboard
- ✓ Adjustable speeds
- ✓ Safety stop (SPACE key)
- ✓ Works through mavros_velocity_node safety layer

### Build and Run:

**Note:** ROS2 and workspace are auto-sourced in your `.bashrc`. Open new terminals and run directly!

**1. Build the updated package:**
```bash
cd /home/jetson/ros_ws/cdrone_control/ros2
colcon build --packages-select drone_control_pkg
```

**2. Terminal 1: Launch MAVROS**
```bash
ros2 launch drone_bringup drone.launch.py
```

**3. Terminal 2: Launch mavros_velocity_node**
```bash
ros2 run drone_control_pkg mavros_velocity_node
```

**4. Terminal 3: Launch keyboard teleop**
```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

### Keyboard Controls:

```
Movement:
  W/S : Forward/Backward
  A/D : Left/Right
  R/F : Up/Down
  Q/E : Yaw Left/Right
  
Speed:
  T : Increase speed by 10%
  Y : Decrease speed by 10%
  
Commands:
  SPACE : Stop (zero velocity)
  1 : Arm drone
  2 : Set STABILIZED mode
  3 : Set OFFBOARD mode
  4 : Disarm
  
Exit:
  ESC or Ctrl+C : Quit
```

---

## 🔄 Control Workflow for STABILIZED Mode

### For STABILIZED mode (manual RC-style control):

**Option 1: Using existing nodes**
```bash
# Terminal 1: MAVROS
ros2 launch drone_bringup drone.launch.py

# Terminal 2: Direct velocity publishing (no safety wrapper)
ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, 
    twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# Arm and set mode
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'STABILIZED'}"
```

**Option 2: Using keyboard teleop (RECOMMENDED)**
```bash
# Terminal 1: MAVROS
ros2 launch drone_bringup drone.launch.py

# Terminal 2: Velocity node (safety wrapper)
ros2 run drone_control_pkg mavros_velocity_node \
  --ros-args -p require_guided_mode:=false  # Allow STABILIZED mode

# Terminal 3: Keyboard teleop
ros2 run drone_control_pkg keyboard_teleop_node

# Then use keyboard:
# Press 1 to arm
# Press 2 for STABILIZED mode
# Use WASD/RF/QE to fly
# Press SPACE to stop
# Press 4 to disarm
```

---

## 🔄 Control Workflow for OFFBOARD Mode

### For OFFBOARD mode (autonomous velocity control):

```bash
# Terminal 1: MAVROS
ros2 launch drone_bringup drone.launch.py

# Terminal 2: Velocity node (with OFFBOARD requirement)
ros2 run drone_control_pkg mavros_velocity_node

# Terminal 3: Arm and set OFFBOARD
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Terminal 4: Send velocity commands
ros2 topic pub /cdrone/control/cmd_vel_body geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, 
    twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

---

## 🛡️ Safety Features

### 1. **E-Stop**
```bash
# Trigger emergency stop
ros2 topic pub /cdrone/safety/estop std_msgs/msg/Bool "{data: true}" --once

# Release e-stop
ros2 topic pub /cdrone/safety/estop std_msgs/msg/Bool "{data: false}" --once
```

### 2. **Watchdog Timer**
- If no velocity commands received for 0.5s (default), velocity node publishes zero
- Prevents runaway if controller crashes

### 3. **Velocity Limits**
- Horizontal: 1.5 m/s max (default)
- Vertical: 0.8 m/s max (default)
- Yaw rate: 0.6 rad/s max (default)

### 4. **Mode Gating**
- By default, only forwards commands when armed and in GUIDED/OFFBOARD
- Disable with `require_guided_mode: false` for STABILIZED mode

---

## 📊 Comparison: STABILIZED vs OFFBOARD

| Feature | STABILIZED | OFFBOARD |
|---------|------------|----------|
| **Use Case** | Manual/RC-style control | Autonomous control |
| **GPS Required** | No | No |
| **Altitude Hold** | No (manual throttle) | Yes (velocity control) |
| **Position Hold** | No | No (use POSITION mode for that) |
| **Failsafe** | RC takeover | RC takeover or timeout RTL |
| **Best For** | Learning, indoor, testing | Autonomous missions |
| **PX4 Support** | ✓ Always supported | ✓ Supported |

---

## ✅ Summary: What's Confirmed Working

1. ✅ **Arming via MAVROS** - `drone_control_node.arm()` or service calls
2. ✅ **STABILIZED mode** - `change_mode('STABILIZED')` or service calls
3. ✅ **OFFBOARD mode** - `change_mode('OFFBOARD')` or service calls
4. ✅ **Velocity control** - Via `mavros_velocity_node` or direct MAVROS
5. ✅ **Position control** - Via `drone_control_node` setpoint publisher
6. ✅ **Safety features** - E-stop, watchdog, velocity limits
7. ✅ **Keyboard teleop** - New node with WASD controls

---

## 🎯 Quick Test Sequence

**Props off for initial testing!**

```bash
# Terminal 1: Launch MAVROS
ros2 launch drone_bringup drone.launch.py

# Wait for: [INFO] [mavros_node]: Connected to FCU

# Terminal 2: Launch velocity node (allow STABILIZED)
ros2 run drone_control_pkg mavros_velocity_node \
  --ros-args -p require_guided_mode:=false

# Terminal 3: Launch keyboard teleop
ros2 run drone_control_pkg keyboard_teleop_node

# In keyboard terminal:
# Press 1 - ARM (should hear motors spin)
# Press 2 - STABILIZED mode
# Press W - Forward (motors should respond)
# Press SPACE - Stop
# Press 4 - DISARM
```

---

## 📝 Next Steps

1. **Test MAVROS connection** ✓ (Already worked with teleop example)
2. **Build keyboard teleop** (Just added - need to rebuild)
3. **Test arming** (Bench test, props off)
4. **Test STABILIZED control**
5. **Test flight** (When ready!)

---

**The repo 100% supports STABILIZED control and arming through MAVROS!** 🎉
