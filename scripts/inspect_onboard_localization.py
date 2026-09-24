#!/usr/bin/env python3
"""Read PX4 parameters/telemetry and print an operator-reviewed ARK Flow diff.

Never sends PARAM_SET, arming, mode, origin, home, or reboot commands. Optional
backup and command output are local artifacts, not commands executed on the FCU.
"""

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import re
import struct
import time

from pymavlink import mavutil
import yaml


DEFAULT_PROFILE = (Path(__file__).resolve().parents[1] / "ros2/src/drone_bringup/config"
                   / "onboard_optical_flow_profile.yaml")
RETAINED_PARAMS = [
    "SENS_FLOW_ROT", "EKF2_OF_POS_X", "EKF2_OF_POS_Y", "EKF2_OF_POS_Z",
    "EKF2_RNG_POS_X", "EKF2_RNG_POS_Y", "EKF2_RNG_POS_Z", "EKF2_OF_QMIN",
    "EKF2_MAG_TYPE", "COM_ARM_WO_GPS", "MPC_XY_P",
]


def load_parameters(path):
    document = yaml.safe_load(Path(path).read_text())
    parameters = document.get("parameters", {})
    if not parameters:
        raise ValueError("profile has no parameters")
    for name, value in parameters.items():
        if (not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", name)
                or isinstance(value, bool) or not isinstance(value, (float, int))
                or not math.isfinite(value)):
            raise ValueError(f"invalid parameter: {name}")
    return parameters


def parameter_commands(parameters):
    lines = [f"param set {name} {value:.9g}" for name, value in parameters.items()]
    return "\n".join(lines + ["param save"])


def decode_parameter(msg, bytewise):
    value, kind = msg.param_value, msg.param_type
    if kind == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        if not math.isfinite(value):
            raise ValueError(f"non-finite value for {msg.param_id}")
        return float(value)
    formats = {1: "B", 2: "b", 3: "H", 4: "h", 5: "I", 6: "i"}
    if kind not in formats:
        raise ValueError(f"unsupported MAVLink parameter type {kind}")
    if not bytewise:
        return int(value)
    fmt = "<" + formats[kind]
    return struct.unpack(fmt, struct.pack("<f", value)[:struct.calcsize(fmt)])[0]


def receive(connection, message_type, target, timeout, predicate=lambda msg: True):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = connection.recv_match(type=message_type, blocking=True,
                                    timeout=min(0.2, max(0, deadline - time.monotonic())))
        if (msg is not None and (msg.get_srcSystem(), msg.get_srcComponent()) == target
                and predicate(msg)):
            return msg
    return None


def read_parameter(connection, name, target, bytewise):
    for _ in range(2):
        connection.mav.param_request_read_send(*target, name.encode("ascii"), -1)
        msg = receive(connection, "PARAM_VALUE", target, 1.0,
                      lambda msg: str(msg.param_id).rstrip("\x00") == name)
        if msg is not None:
            return decode_parameter(msg, bytewise)
    raise TimeoutError(f"parameter {name} unavailable")


def request_message(connection, target, message_id):
    connection.mav.command_long_send(
        *target, mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, message_id, 0, 0, 0, 0, 0, 0)


def inspect(args, parameters):
    target = (args.target_system, 1)
    connection = mavutil.mavlink_connection(args.port, source_system=249, source_component=191)
    try:
        connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        heartbeat = receive(connection, "HEARTBEAT", target, 5.0)
        if heartbeat is None:
            raise TimeoutError("no FCU heartbeat through mavlink-router")
        request_message(connection, target, 148)
        version = receive(connection, "AUTOPILOT_VERSION", target, 3.0)
        if version is None:
            raise TimeoutError("cannot verify firmware version/parameter encoding")
        # Older pymavlink calls the bytewise capability PARAM_UNION. The MAVLink
        # wire bit remains 16; accept only the advertised PX4 encoding here.
        bytewise = bool(version.capabilities & 16)
        c_cast = bool(version.capabilities & 131072)
        if not bytewise and not c_cast:
            raise ValueError("FCU did not advertise its integer parameter encoding")
        firmware = tuple((version.flight_sw_version >> shift) & 255 for shift in (24, 16, 8))
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"FCU {target}: PX4 {'.'.join(map(str, firmware))}; armed={armed}")
        print("Read-only inspection; no configuration or flight commands are sent.")
        current = {name: read_parameter(connection, name, target, bytewise) for name in parameters}
        mismatches = []
        for name, desired in parameters.items():
            actual = current[name]
            matches = math.isclose(actual, desired, rel_tol=1e-6, abs_tol=1e-6)
            label = "OK  " if matches else "DIFF"
            print(f"{label} {name}: current={actual:.9g} desired={desired:.9g}")
            if not matches:
                mismatches.append(name)
        print("Mounting/heading/quality/controller settings retained (verify on the airframe):")
        for name in RETAINED_PARAMS:
            try:
                print(f"  {name}={read_parameter(connection, name, target, bytewise):.9g}")
            except TimeoutError as exc:
                print(f"  {exc}")
        if args.backup:
            document = {
                "name": "onboard_localization_previous_values",
                "captured_utc": datetime.now(timezone.utc).isoformat(),
                "firmware": list(firmware),
                "description": "Original values of all parameters touched by the selected profile.",
                "parameters": current,
            }
            # Never overwrite an earlier rollback snapshot.
            with Path(args.backup).open("x", encoding="utf-8") as stream:
                yaml.safe_dump(document, stream, sort_keys=False)
            print(f"Saved rollback profile: {args.backup}")
        messages = {}
        for message_id in (100, 132, 230):
            request_message(connection, target, message_id)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            msg = receive(connection, ["OPTICAL_FLOW_RAD", "DISTANCE_SENSOR", "ESTIMATOR_STATUS"],
                          target, 0.3)
            if msg:
                messages[msg.get_type()] = msg
        for name in ("OPTICAL_FLOW_RAD", "DISTANCE_SENSOR", "ESTIMATOR_STATUS"):
            msg = messages.get(name)
            detail = msg.to_dict() if msg else "not observed (check PX4 listener/telemetry streams)"
            print(f"{name}: {detail}")
        print("A parameter match does not prove sensor fusion or flight readiness. "
              "Reboot after changes; verify cs_opt_flow and use position_hover_check.")
        if firmware[:2] != (1, 16) or heartbeat.autopilot != mavutil.mavlink.MAV_AUTOPILOT_PX4:
            print("BLOCKED: profile is scoped to PX4 1.16; verify settings for this firmware.")
            return 2
        return 2 if mismatches else 0
    finally:
        connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--port", default="udpout:127.0.0.1:14550")
    parser.add_argument("--target-system", type=int, default=1)
    parser.add_argument("--backup", type=Path,
                        help="Save original touched values to a NEW local YAML file")
    parser.add_argument("--print-commands", action="store_true",
                        help="Offline: print PX4 console commands for review; do not connect")
    args = parser.parse_args()
    try:
        parameters = load_parameters(args.profile)
        if args.print_commands:
            print(parameter_commands(parameters))
            return 0
        return inspect(args, parameters)
    except (ValueError, OSError, TimeoutError) as exc:
        print(f"Inspection failed: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
