#!/usr/bin/env python3
"""Minimal PARAM_SET via raw UDP MAVLink (no pymavlink needed)."""
import socket
import struct
import sys
import time

SYSID = 255
COMPID = 190
TGT_SYS = 1
TGT_COMP = 1

def crc_x25(buf):
    crc = 0xffff
    for b in buf:
        tmp = b ^ (crc & 0xff)
        tmp = (tmp ^ (tmp << 4)) & 0xff
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xffff
    return crc

# PARAM_SET (msgid=23) MAVLink1 structure
# target_system, target_component, param_id[16], param_value(float), param_type(uint8)
# Payload = 1+1+16+4+1 = 23 bytes
def make_param_set(param_id: str, value: float):
    msgid = 23
    pid = param_id.encode()[:16].ljust(16, b'\x00')
    payload = struct.pack('<BBBBf', TGT_SYS, TGT_COMP, 0, 0, value)
    # Actually MAVLink PARAM_SET v1 payload: target_system(1), target_component(1), param_id(16), param_value(4f), param_type(1u)
    payload = struct.pack('>ff', 0, 0)  # placeholder - need to do this correctly
    # MAVLink1 PARAM_SET: param_value(float), target_system, target_component, param_id[16], param_type
    payload = struct.pack('<f', value) + bytes([TGT_SYS, TGT_COMP]) + pid + bytes([9])  # 9=MAV_PARAM_TYPE_INT32
    length = len(payload)
    seq = 0
    header = bytes([0xFE, length, seq, SYSID, COMPID, msgid])
    crc_seed = bytes([length, seq, SYSID, COMPID, msgid]) + payload + bytes([0xF0])
    crc = crc_x25(crc_seed)
    return header + payload + struct.pack('<H', crc)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 14548))
sock.settimeout(1.0)

param = sys.argv[1] if len(sys.argv) > 1 else 'COM_RC_IN_MODE'
val = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

pkt = make_param_set(param, val)
for _ in range(3):
    sock.sendto(pkt, ('127.0.0.1', 14550))
    print(f'Sent PARAM_SET {param}={val}')
    time.sleep(0.3)
