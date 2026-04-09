#!/usr/bin/env python3
"""Send PARAM_SET directly to PX4 via mavlink_router UDP."""
import sys
import time
from pymavlink import mavutil

param_name = sys.argv[1] if len(sys.argv) > 1 else 'COM_RC_IN_MODE'
param_val = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

m = mavutil.mavlink_connection('udp:127.0.0.1:14550', source_system=2, source_component=190)
print("Waiting for heartbeat...")
m.wait_heartbeat(timeout=5)
print(f"Connected to sysid={m.target_system}")

# Send PARAM_REQUEST_READ first to get current value
m.mav.param_request_read_send(
    m.target_system, m.target_component,
    param_name.encode(), -1
)
msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
if msg:
    pid = msg.param_id.rstrip('\x00')
    if pid == param_name:
        print(f"Current {param_name} = {msg.param_value}")

# Set the param
m.mav.param_set_send(
    m.target_system, m.target_component,
    param_name.encode(), param_val,
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)
print(f"Sent PARAM_SET {param_name}={param_val}")

# Wait for ACK
for _ in range(20):
    msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=0.5)
    if msg:
        pid = msg.param_id.rstrip('\x00')
        if pid == param_name:
            print(f"ACK: {param_name} = {msg.param_value}")
            break
