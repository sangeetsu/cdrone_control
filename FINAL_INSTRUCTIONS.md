# MAVROS Keyboard Teleop - Final Instructions

**System:** Jetson Orin Nano + ARK PAB + ARKV6X + PX4 1.16.0  
**Method:** MAVROS (MAVLink) - No external modes needed  
**Status:** Ready for props-off testing

---

## 📋 Quick Reference

### What We Built
- **Keyboard teleop node** for MAVROS drone control
- **Safety wrapper** (mavros_velocity_node) with watchdog and e-stop
- **Auto-sourcing workspace** - no manual setup needed per terminal

### Why MAVROS Instead of ros2_px4_teleop_example?
- ✅ Works with standard PX4 (no external modes needed)
- ✅ MAVLink is stable across PX4 versions
- ✅ Simpler setup, fewer dependencies
- ✅ Your PX4 build lacks COM_EXT_MODES parameter

---

## 🚀 Daily Operation (After Initial Setup)

### Start System (3 Terminals)

**Terminal 1: Launch MAVROS**
```bash
ros2 launch drone_bringup drone.launch.py
```
*Wait for: "Connected to FCU"*

**Terminal 2: Launch Safety Wrapper**
```bash
ros2 run drone_control_pkg mavros_velocity_node
```
*Should start with no errors*

**Terminal 3: Launch Keyboard Teleop**
```bash
ros2 run drone_control_pkg keyboard_teleop_node
```
*Keyboard interface appears*

### Control Your Drone

**Movement (in Terminal 3):**
- `W/S` - Forward/Backward
- `A/D` - Left/Right  
- `R/F` - Up/Down
- `Q/E` - Yaw Left/Right

**Commands:**
- `1` - ARM
- `4` - DISARM
- `2` - STABILIZED mode (default flight mode)
- `3` - OFFBOARD mode (for velocity control)
- `SPACE` - EMERGENCY STOP
- `T/Y` - Decrease/Increase speed
- `X` - Exit

### Stop System
Just press `Ctrl+C` in each terminal (or close them).

---

## 📁 File Locations

### Key Configuration Files
- **MAVROS config**: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/config/px4_config.yaml`
- **Launch files**: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/launch/`
- **Keyboard node**: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/keyboard_teleop_node.py`

### Documentation Files
- **[props_off_test.md](props_off_test.md)** - Detailed bench testing procedure ⚠️ START HERE
- **[MAVROS_CONTROL_GUIDE.md](MAVROS_CONTROL_GUIDE.md)** - Complete MAVROS capabilities reference
- **[JETSON_PX4_SETUP.md](JETSON_PX4_SETUP.md)** - Initial setup guide (already done)

---

## ⚠️ First Time Testing? READ THIS

**Before first test:**
1. ✅ Read **[props_off_test.md](props_off_test.md)** completely
2. ✅ Remove ALL propellers
3. ✅ Follow each test phase in order
4. ✅ Verify each phase succeeding before continuing

**Do NOT skip to props-on flight testing without completing bench tests!**

---

## 🔧 Troubleshooting Quick Reference

### MAVROS Won't Connect
```bash
# Check mavlink-router service (should be active)
systemctl status mavlink-router

# Restart mavlink-router if needed
sudo systemctl restart mavlink-router

# Check USB connection to FC
ls -l /dev/serial/by-id/usb-ARK*

# Verify MAVROS config uses UDP
cat /home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/config/apm_params.yaml
# Should show: fcu_url: "udp://:14540@127.0.0.1:14550"
```

### Can't Build Package
```bash
cd /home/jetson/ros_ws/cdrone_control/ros2
colcon build --packages-select drone_control_pkg
# Workspace auto-sources on new terminal
```

### Keyboard Not Responding
- Click on Terminal 3 (keyboard teleop) to give it focus
- Check node is running: `ros2 node list | grep keyboard`

### "Watchdog Timeout" Errors
- Keep pressing keys (or hold a direction)
- Commands must be sent regularly (default 1 second timeout)
- Emergency stop clears all commands

---

## 🖥️ System Architecture

```
┌─────────────────┐
│  PX4 (ARKV6X)   │
│  Flight Control │
└────────┬────────┘
         │ USB (2Mbps)
         │ MAVLink Protocol
┌────────▼────────────┐
│  mavlink-router     │ <- systemd service (always running)
│  (MAVLink Multiplex)│
└─────────┬───────────┘
          │
          ├─► UDP 14550 ──► QGroundControl (monitoring)
          │
          └─► UDP 14540 ──┐
                          │
                ┌─────────▼─────────┐
                │  MAVROS Node      │ <- ros2 launch drone_bringup drone.launch.py
                │  (MAVLink↔ROS2)   │
                └─────────┬─────────┘
                          │ ROS2 Topics
                          ├─► /mavros/state (status)
                          ├─► /mavros/cmd/arming (arm/disarm)
                          ├─► /mavros/set_mode (mode changes)
                          └─► /mavros/setpoint_velocity/cmd_vel (velocity cmds)
                          │
                ┌─────────▼─────────────┐
                │ mavros_velocity_node  │ <- ros2 run drone_control_pkg mavros_velocity_node
                │ (Safety Wrapper)      │
                └─────────┬─────────────┘
                          │ /cdrone/control/cmd_vel_body
                          │ (watchdog, e-stop, velocity limits)
                ┌─────────▼─────────────┐
                │ keyboard_teleop_node  │ <- ros2 run drone_control_pkg keyboard_teleop_node
                │ (WASD Control)        │
                └───────────────────────┘
```

**Key Benefit:** QGC and MAVROS both work simultaneously via mavlink-router!

---

## 📡 ROS2 Topics Reference

### Published by keyboard_teleop_node
- `/cdrone/control/cmd_vel_body` - Velocity commands (Twist)

### Subscribed by keyboard_teleop_node
- `/mavros/state` - Flight controller state (armed, mode, connected)

### Services called by keyboard_teleop_node
- `/mavros/cmd/arming` - Arm/disarm drone
- `/mavros/set_mode` - Change flight mode

### Monitor with:
```bash
# Check connection status
ros2 topic echo /mavros/state --once

# Watch velocity commands
ros2 topic echo /cdrone/control/cmd_vel_body

# See all MAVROS topics
ros2 topic list | grep mavros
```

---

## 🛠️ Advanced Configuration

### MAVROS Connection (via mavlink-router)
Edit: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/config/apm_params.yaml`
```yaml
fcu_url: "udp://:14540@127.0.0.1:14550"  # UDP to mavlink-router
```

**Current setup uses mavlink-router** - QGC and MAVROS both work via UDP!

### Adjust Velocity Limits
Edit: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_control_pkg/drone_control_pkg/keyboard_teleop_node.py`
```python
self.speed_scale = 0.5  # Default speed (m/s)
self.max_speed = 2.0    # Maximum speed
self.min_speed = 0.1    # Minimum speed
self.speed_increment = 0.1  # T/Y key adjustment
```

### Modify Watchdog Timeout
Edit: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_control_pkg/launch/mavros_velocity.launch.py`
```python
{'watchdog_timeout': 1.0}  # Increase if getting false timeouts
```

**After any changes:**
```bash
cd /home/jetson/ros_ws/cdrone_control/ros2
colcon build --packages-select drone_control_pkg
# Open new terminal - auto-sources updated build
```

---

## 🔄 Workspace Auto-Sourcing

Your `~/.bashrc` automatically sources:
```bash
source /opt/ros/humble/setup.bash
source /home/jetson/ros_ws/cdrone_control/ros2/install/setup.bash
```

**No need to manually source in new terminals!**

To disable auto-sourcing:
```bash
nano ~/.bashrc
# Comment out or delete the source lines
# Restart terminal
```

---

## 🧪 Testing Workflow

### Phase 1: Bench Testing (Props Off) ⚠️
**File: [props_off_test.md](props_off_test.md)**

1. ✅ MAVROS connection test
2. ✅ Mode change test (STABILIZED)
3. ✅ Arming test
4. ✅ Velocity command test
5. ✅ Emergency stop test
6. ✅ Speed scaling test

**Goal:** Verify all software working, motors respond, no crashes

---

### Phase 2: Outdoor Manual Flight (Props On, No Autonomy)
**NOT YET - Need separate guide**

1. ⚠️ Open area, no obstacles
2. ⚠️ Install propellers (correct directions!)
3. ⚠️ RC transmitter ready (safety pilot)
4. ⚠️ Manual mode flight test
5. ⚠️ Verify GPS lock
6. ⚠️ Practice emergency RC takeover

**Goal:** Verify drone mechanically sound, safe to fly

---

### Phase 3: Keyboard Teleop Flight (Props On, Low Altitude)
**NOT YET - Need separate guide + Phase 2 complete**

1. ⚠️ Start in manual, take off to 1-2m
2. ⚠️ Land, switch to STABILIZED + OFFBOARD
3. ⚠️ Test keyboard controls at low altitude
4. ⚠️ Practice emergency RC takeover
5. ⚠️ Gradually increase altitude/confidence

**Goal:** Verify MAVROS control works in real flight

---

## 📚 Additional Resources

### Codebase Overview
- **drone_bringup**: Launch files for MAVROS and full stack
- **drone_control_pkg**: Keyboard teleop, safety wrapper, arm/mode control
- **drone_behavior_pkg**: Autonomous behaviors (not used in teleop)
- **drone_vision_pkg**: Vision processing (not used in teleop)
- **drone_light_pkg**: LED control (not used in teleop)

### Control Modes Available
- **MANUAL**: RC control only (default)
- **STABILIZED**: Self-leveling, good for teleop
- **ALTITUDE**: Holds altitude (needs barometer)
- **POSITION**: Holds position (needs GPS)
- **OFFBOARD**: External control via MAVROS (needed for velocity commands)
- **GUIDED**: High-level waypoint commands

**For keyboard teleop: Use STABILIZED + OFFBOARD**

### MAVROS Documentation
- Official docs: http://wiki.ros.org/mavros
- PX4 + MAVROS: https://docs.px4.io/main/en/ros2/mavros_installation.html

---

## 🐛 Common Issues & Solutions

### "Registration failed" Error
- **Cause**: You're trying to run ros2_px4_teleop_example (old approach)
- **Solution**: Use MAVROS method instead (this guide)

### Package Not Found After Build
```bash
# Open NEW terminal (to pick up new build)
ros2 pkg list | grep drone_control_pkg
# Should see it listed
```

### Motors Don't Spin During Bench Test
- ✅ Check armed (press `1`)
- ✅ Check mode is OFFBOARD (press `3`) for velocity control
- ✅ STABILIZED alone won't spin motors without RC throttle
- ✅ Battery connected (ESCs need power)

### QGC and MAVROS Conflict
- **Best**: QGC on TELEM2 (`/dev/ttyTHS1`), MAVROS on USB (`/dev/ttyACM0`)
- **Or**: Don't run QGC during MAVROS testing
- **Never**: Both on same serial port simultaneously

---

## ✅ Pre-Flight Checklist (Props Off Testing)

Hardware:
- [ ] **ALL PROPELLERS REMOVED**
- [ ] USB connected to Jetson
- [ ] Battery connected (if testing motors)
- [ ] Flight controller powered on

Software:
- [ ] Workspace sourced (auto in new terminal)
- [ ] No error in: `ros2 pkg list | grep drone_bringup`
- [ ] No conflicting processes: `ps aux | grep -E "micro-xrce|mavlink"`

Testing:
- [ ] Read [props_off_test.md](props_off_test.md) completely
- [ ] Understand emergency stop procedure (SPACE key)
- [ ] Ready to disconnect battery if needed

---

## 🎯 Next Steps

**Right Now:**
1. Read [props_off_test.md](props_off_test.md)
2. Remove propellers
3. Run through Phase 1-7 bench tests
4. Document any issues encountered

**After Successful Bench Tests:**
1. Review test results
2. Fix any issues found
3. Plan outdoor manual flight test
4. Schedule safe flight testing conditions

**Eventually:**
1. Full autonomy stack testing (has vision, behaviors)
2. Waypoint navigation
3. Object detection and avoidance

---

## 📞 Need Help?

**Check logs:**
```bash
cd /home/jetson/ros_ws/cdrone_control/ros2/log/latest
ls -la
cat */events.log | grep ERROR
```

**Verify system:**
```bash
# ROS2 workspace loaded?
ros2 pkg list | grep drone_bringup

# MAVROS connected?
ros2 topic echo /mavros/state --once

# Nodes running?
ros2 node list

# Topics flowing?
ros2 topic hz /mavros/state
```

**Reset everything:**
```bash
# Kill all ROS nodes
pkill -f ros2

# Restart from Terminal 1-3 procedure above
```

---

## 🎉 You're Ready!

Everything is set up and ready for testing:
- ✅ Workspace built
- ✅ Auto-sourcing configured  
- ✅ Keyboard teleop compiled
- ✅ Documentation complete

**Next action: Open [props_off_test.md](props_off_test.md) and start Phase 1!**

**Stay safe, remove props, and happy testing! 🚁**
