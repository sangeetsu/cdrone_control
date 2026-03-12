# Props-Off Bench Testing Guide



# Terminal 1
ros2 launch drone_bringup drone.launch.py

# Terminal 2  
ros2 run drone_control_pkg mavros_velocity_node

# Terminal 3
ros2 run drone_control_pkg keyboard_teleop_node



**⚠️ SAFETY FIRST: REMOVE ALL PROPELLERS BEFORE TESTING ⚠️**

This guide walks through safe bench testing of the MAVROS keyboard teleop system.

---

## QGroundControl and MAVROS Together

**Can they coexist?** ✅ **YES! Already configured!**

### Your System Uses mavlink-router (Already Running)
**mavlink-router** multiplexes MAVLink from FC USB to multiple UDP endpoints:
- FC USB @ 2Mbps → mavlink-router
- mavlink-router → UDP 127.0.0.1:14550 (GCS - for QGC)
- mavlink-router → UDP 127.0.0.1:14540 (MAVROS)

**Both QGC and MAVROS work simultaneously!** 🎉

### How to Use:
1. **QGC**: Connect to UDP 127.0.0.1:14550 (default)
   - Should auto-connect if QGC is already set up
2. **MAVROS**: Already configured for UDP (check below)

**No conflicts, no serial port issues, everything just works!**

---

## Pre-Flight Checklist

### Hardware Prep
- [ ] **REMOVE ALL 4 PROPELLERS** (cannot stress this enough!)
- [ ] Flight controller connected via USB (`/dev/ttyACM0`)
- [ ] Battery connected (for motor output, if safe)
- [ ] Jetson powered on
- [ ] Good ventilation (motors may spin)

### Software Prep
- [ ] Workspace auto-sourced (open new terminal, check `ros2 pkg list | grep drone_bringup`)
- [ ] mavlink-router running: `systemctl status mavlink-router` (should be active)
- [ ] QGC can run simultaneously (optional for monitoring)

---

## Test Procedure

### Phase 1: MAVROS Connection Test

**Terminal 1: Launch MAVROS**
```bash
ros2 launch drone_bringup drone.launch.py
```

**Expected output:**
```
[INFO] [mavros_node]: FCU: PX4 Autopilot v1.16.0
[INFO] [mavros_node]: FCU: ARKV6X
[INFO] [mavros_node]: Connected to FCU
```

**Terminal 2: Verify MAVROS Topics**
```bash
# Check state topic
ros2 topic echo /mavros/state --once

# Should show: connected: True, armed: False, mode: "MANUAL" or similar
```

**✓ Success Criteria:**
- MAVROS connects without errors
- `/mavros/state` shows `connected: True`
- Flight controller heartbeat visible in logs

**❌ Troubleshooting:**
- "Device opening failed": Check USB cable, try `sudo chmod 666 /dev/ttyACM0`
- "FCU not responding": Reboot flight controller
- Permission denied: Add user to dialout group: `sudo usermod -a -G dialout $USER`, then reboot

---

### Phase 2: QGC Monitoring (Optional)

**If using QGC for monitoring:**
1. Launch QGC
2. Connect to `/dev/ttyTHS1` (TELEM2)
3. Should see vehicle status, no conflicts with MAVROS on USB
4. Keep QGC open to monitor mode changes

---

### Phase 3: Launch Control Nodes

**Terminal 3: Launch mavros_velocity_node**
```bash
ros2 run drone_control_pkg mavros_velocity_node
```

**Expected output:**
```
[INFO] [mavros_velocity_node]: Safety wrapper initialized
[INFO] [mavros_velocity_node]: Watchdog timeout: 1.0s
```

**Terminal 4: Launch keyboard_teleop_node**
```bash
ros2 run drone_control_pkg keyboard_teleop_node
```

**Expected output:**
```
╔══════════════════════════════════════════════════════════╗
║           MAVROS Drone Keyboard Teleop                   ║
╠══════════════════════════════════════════════════════════╣
║ Movement:                                                ║
║   W/S - Forward/Backward                                 ║
║   A/D - Left/Right                                       ║
║   R/F - Up/Down                                          ║
║   Q/E - Yaw Left/Right                                   ║
║                                                          ║
║ Commands:                                                ║
║   1 - ARM  |  4 - DISARM                                ║
║   2 - STABILIZED MODE                                    ║
║   3 - OFFBOARD MODE                                      ║
║   SPACE - EMERGENCY STOP                                 ║
║   T/Y - Decrease/Increase Speed                          ║
║   X - Exit                                               ║
╚══════════════════════════════════════════════════════════╝

Current speed scale: 0.50 m/s
```

**✓ Success Criteria:**
- Both nodes start without errors
- Keyboard interface displays correctly
- No ROS warnings about missing topics

---

### Phase 4: Mode Change Test (Bench, Props Off)

**In keyboard teleop terminal:**

1. **Press `2` - Switch to STABILIZED**
   - Watch terminal output: "Mode changed to: STABILIZED"
   - In QGC (if connected): Flight mode should change to "Stabilized"
   - **Expected:** No errors, mode changes successfully

2. **Check flight controller reaction:**
   - LEDs may change pattern
   - No abnormal sounds/heat from FC

**✓ Success Criteria:**
- Mode changes without errors
- `/mavros/state` shows updated mode: `ros2 topic echo /mavros/state --once`
- QGC reflects mode change (if monitoring)

**❌ Troubleshooting:**
- "Command failed": Check MAVROS connection, restart if needed
- Mode doesn't change: May need MANUAL mode first, or check PX4 parameters

---

### Phase 5: Arming Test (Bench, Props Off, Battery Connected)

**⚠️ CRITICAL: VERIFY PROPELLERS REMOVED ⚠️**

**In keyboard teleop terminal:**

1. **Press `1` - ARM**
   - Watch terminal: "Arming requested..."
   - FC should play arming sound (if speaker connected)
   - Motors may make small "ready" sounds

**Expected behaviors:**
- Terminal: "Arming successful" or "Armed: True"
- QGC (if monitoring): Shows "ARMED"
- `/mavros/state` topic: `armed: True`
- Motors: May beep/hum but should NOT spin yet

**✓ Success Criteria:**
- Arming succeeds without errors
- Flight controller LED changes (typically solid or different pattern)
- No unexpected motor movement

**⚠️ If motors spin unexpectedly:**
- **Press `4` immediately to DISARM**
- Or press **SPACE for EMERGENCY STOP**
- Check that you're NOT in OFFBOARD mode (should be STABILIZED)
- In STABILIZED, motors shouldn't spin without throttle input

2. **Press `4` - DISARM**
   - FC should play disarming sound
   - LEDs return to disarmed pattern
   - Terminal: "Disarmed: True"

**✓ Success Criteria:**
- Clean arm → disarm cycle
- No persistent errors

---

### Phase 6: Velocity Command Test (Armed, Props Off)

**⚠️ DOUBLE-CHECK: PROPELLERS REMOVED ⚠️**

**Setup:**
1. Press `2` - STABILIZED mode
2. Press `1` - ARM
3. Wait for successful arm

**Test velocity commands:**

1. **Press `W` (Forward thrust)**
   - Terminal shows: "Vel: x=0.5, y=0.0, z=0.0, yaw=0.0"
   - Motors: May spin UP (EXPECTED - props removed, safe)
   - Release `W` - Motors stop/slow down

2. **Press `A` (Left thrust)**
   - Terminal shows: "Vel: x=0.0, y=-0.5, z=0.0, yaw=0.0"
   - Motors: May spin differently for lateral movement

3. **Press `R` (Upward thrust)**
   - Terminal shows: "Vel: x=0.0, y=0.0, z=0.5, yaw=0.0"
   - Motors: Should spin UP (all motors increase)

4. **Press `SPACE` (Emergency stop)**
   - Terminal: "EMERGENCY STOP ACTIVATED"
   - All velocity commands stop immediately
   - Motors return to idle

**✓ Success Criteria:**
- Velocity commands published successfully
- Watchdog doesn't trigger (commands sent regularly)
- Motor responses correspond to commands (spin patterns change)
- Emergency stop works instantly

**Expected motor behaviors (props off):**
- Forward (W): Front motors slower, rear motors faster (pitch forward)
- Left (A): Right motors faster, left motors slower (roll left)
- Up (R): All motors increase RPM
- Yaw (Q/E): Diagonal pairs change differentially

5. **Press `4` - DISARM when done**

---

### Phase 7: Speed Scaling Test

**In keyboard teleop terminal:**

1. **Press `T` multiple times** - Decrease speed
   - Terminal shows decreasing scale: "0.40 m/s", "0.30 m/s", etc.
   - Min: 0.10 m/s

2. **Press `Y` multiple times** - Increase speed
   - Terminal shows increasing scale: "0.60 m/s", "0.70 m/s", etc.
   - Max: 2.00 m/s

3. **Test with different speeds:**
   - Set to 0.20 m/s
   - Press `W` - Should be gentler motor response
   - Set to 1.00 m/s
   - Press `W` - Should be stronger motor response

**✓ Success Criteria:**
- Speed scaling works smoothly
- Commands scale appropriately with setting
- No crashes or overflow errors

---

## Post-Test Checklist

### Successful Test Results
- [ ] MAVROS connected successfully
- [ ] Mode changes work (STABILIZED)
- [ ] Arming/disarming works cleanly
- [ ] Velocity commands reach motors
- [ ] Emergency stop responds instantly
- [ ] Speed scaling functions correctly
- [ ] No persistent errors or warnings

### Cleanup
```bash
# Stop all nodes (Ctrl+C in each terminal)
# Or from any terminal:
pkill -f ros2
```

### Known Issues to Note
- Any errors during testing (write them down)
- Unexpected behaviors (motor patterns, timing)
- QGC connection stability (if used)

---

## Next Steps After Successful Bench Test

### ✅ If All Tests Pass:
You're ready for **outdoor props-on testing** (different guide needed):
1. Outdoor open area, no obstacles
2. Install propellers (correct direction!)
3. Manual flight test first (no autonomy)
4. Ground effect hover test
5. Then try MAVROS teleop at low altitude

### ❌ If Tests Fail:
**Common issues:**

**MAVROS won't connect:**
- Check mavlink-router: `systemctl status mavlink-router` (should be active)
- Restart mavlink-router: `sudo systemctl restart mavlink-router`
- Check USB cable to FC (mavlink-router needs it)
- Verify PX4 parameter: `SYS_COMP_ID = 1`, `SYS_HITL = 0`
- Check MAVROS config: `/home/jetson/ros_ws/cdrone_control/ros2/src/drone_bringup/config/apm_params.yaml`
  - Should be: `fcu_url: "udp://:14540@127.0.0.1:14550"`

**Can't arm:**
- Check PX4 preflight checks in QGC
- May need GPS fix (indoor = no GPS, disable check: `COM_ARM_WO_GPS = 1`)
- Calibrate sensors (accel, mag, gyro)
- Check battery voltage

**Mode changes fail:**
- Verify mode exists: `ros2 topic echo /mavros/state`
- Check PX4 allows mode: Some modes need GPS, stick calibration
- Try MANUAL first, then STABILIZED

**Motors don't respond:**
- Check arming status
- Verify in correct mode (not MANUAL - needs stick input)
- Check PX4 parameter: `COM_MOT_TEST_EN = 1` (allows motor test)
- May need throttle input in some modes

**Watchdog timeout errors:**
- Keyboard not in focus (click terminal)
- System lag (Jetson CPU overloaded)
- Increase timeout in mavros_velocity_node launch

---

## Safety Reminders

1. **ALWAYS remove propellers for bench testing**
2. **NEVER arm with propellers on indoors**
3. **Keep emergency stop ready** (SPACE key)
4. **Disconnect battery if unexpected behavior occurs**
5. **First outdoor flight: manual mode only, high altitude**
6. **Test emergency stop regularly**
7. **Have a safety pilot with RC transmitter during autonomous flight**

---

## Log Files

Logs are saved to: `/home/jetson/ros_ws/cdrone_control/ros2/log/`

**To review after testing:**
```bash
cd /home/jetson/ros_ws/cdrone_control/ros2/log/latest
ls -la
# Look for error messages in node logs
```

---

## Questions? Debugging?

1. Check MAVROS connection: `ros2 topic hz /mavros/state` (should be ~1 Hz)
2. Check velocity commands: `ros2 topic echo /cdrone/control/cmd_vel_body`
3. Monitor MAVROS logs for errors
4. Use QGC for PX4-side diagnostics
5. Verify ROS2 nodes running: `ros2 node list`

**Ready to test safely! Remove those props and let's verify the system works! 🚁**
