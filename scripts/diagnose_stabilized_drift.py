#!/usr/bin/env python3
"""Read-only PX4 bench diagnostic for stabilized-mode drift."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from pymavlink import mavutil


DEFAULT_ENDPOINTS = (
    "udpin:0.0.0.0:14569",
    "udpin:0.0.0.0:14570",
    "udpin:0.0.0.0:14571",
    "udpout:127.0.0.1:14550",
)

PARAMS_OF_INTEREST = (
    "SENS_BOARD_ROT",
    "SENS_BOARD_X_OFF",
    "SENS_BOARD_Y_OFF",
    "SENS_BOARD_Z_OFF",
    "CAL_ACC0_ID",
    "CAL_ACC0_XOFF",
    "CAL_ACC0_YOFF",
    "CAL_ACC0_ZOFF",
    "CAL_ACC1_ID",
    "CAL_ACC1_XOFF",
    "CAL_ACC1_YOFF",
    "CAL_ACC1_ZOFF",
    "CAL_ACC2_ID",
    "CAL_ACC2_XOFF",
    "CAL_ACC2_YOFF",
    "CAL_ACC2_ZOFF",
    "RC1_TRIM",
    "RC1_DZ",
    "RC2_TRIM",
    "RC2_DZ",
    "RC3_TRIM",
    "RC3_DZ",
    "RC4_TRIM",
    "RC4_DZ",
    "COM_RC_IN_MODE",
    "EKF2_EV_CTRL",
    "EKF2_OF_CTRL",
    "EKF2_HGT_REF",
    "EKF2_MAG_TYPE",
    "MPC_XY_VEL_P_ACC",
    "MPC_XY_VEL_I_ACC",
    "MPC_THR_HOVER",
    "MPC_USE_HTE",
)

MESSAGE_INTERVALS_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU: 20,
    mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_ESTIMATOR_STATUS: 2,
}


@dataclass
class Sample:
    phase: str
    timestamp_s: float
    msg_type: str
    values: dict[str, Any]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only PX4 bench diagnostic for manual/stabilized drift."
    )
    parser.add_argument(
        "--endpoint",
        action="append",
        dest="endpoints",
        help="MAVLink endpoint to try. Can be repeated.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Seconds to sample each phase. Default: 10.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for report files. Default: temp_outputs/stabilized_drift_<stamp>.",
    )
    parser.add_argument(
        "--interactive-sign-test",
        action="store_true",
        help="Prompt for level, nose-down, nose-up, roll-right, and roll-left captures.",
    )
    parser.add_argument(
        "--bias-threshold-deg",
        type=float,
        default=0.2,
        help="Pitch/roll absolute average above this is reported as a level bias.",
    )
    parser.add_argument(
        "--param-timeout",
        type=float,
        default=2.0,
        help="Seconds to wait per parameter read. Default: 2.",
    )
    return parser


def connect(endpoints: list[str]) -> tuple[Any, str, Any]:
    errors: list[str] = []
    for endpoint in endpoints:
        print(f"Probing {endpoint}...")
        try:
            connection = mavutil.mavlink_connection(
                endpoint,
                source_system=250,
                source_component=193,
                autoreconnect=False,
            )
            connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            heartbeat = None
            fallback_heartbeat = None
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline:
                msg = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                if msg is None:
                    continue
                fallback_heartbeat = fallback_heartbeat or msg
                if (
                    msg.type != mavutil.mavlink.MAV_TYPE_GCS
                    and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID
                ):
                    heartbeat = msg
                    connection.target_system = msg.get_srcSystem()
                    connection.target_component = msg.get_srcComponent()
                    break
            heartbeat = heartbeat or fallback_heartbeat
        except Exception as exc:  # noqa: BLE001 - diagnostics should keep probing.
            errors.append(f"{endpoint}: {exc}")
            continue
        if heartbeat:
            return connection, endpoint, heartbeat
        errors.append(f"{endpoint}: no heartbeat")
    raise RuntimeError("Could not connect to PX4:\n" + "\n".join(errors))


def request_message_intervals(connection: Any) -> None:
    for msg_id, hz in MESSAGE_INTERVALS_HZ.items():
        connection.mav.command_long_send(
            connection.target_system,
            connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            1_000_000 / hz,
            0,
            0,
            0,
            0,
            0,
        )


def decode_param_value(value: float, param_type: int) -> float | int:
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)

    raw = struct.pack("<f", float(value))
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack("<B", raw[:1])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack("<b", raw[:1])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack("<H", raw[:2])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack("<h", raw[:2])[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack("<I", raw)[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack("<i", raw)[0]

    return float(value)


def read_param(connection: Any, name: str, timeout_s: float) -> float | int | None:
    connection.mav.param_request_read_send(
        connection.target_system,
        connection.target_component,
        name.encode("ascii"),
        -1,
    )
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        msg = connection.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.25)
        if msg is None:
            continue
        param_id = msg.param_id
        if isinstance(param_id, bytes):
            param_id = param_id.decode("ascii", "ignore")
        if param_id.strip("\x00") == name:
            if msg.get_srcSystem() not in (0, 250):
                connection.target_system = msg.get_srcSystem()
                connection.target_component = msg.get_srcComponent()
            return decode_param_value(msg.param_value, msg.param_type)
    return None


def read_params(connection: Any, timeout_s: float) -> dict[str, float | int | None]:
    return {
        name: read_param(connection, name, timeout_s) for name in PARAMS_OF_INTEREST
    }


def capture_phase(connection: Any, phase: str, duration_s: float) -> list[Sample]:
    samples: list[Sample] = []
    deadline = time.monotonic() + duration_s
    wanted = ["ATTITUDE", "HIGHRES_IMU", "RAW_IMU", "RC_CHANNELS", "ESTIMATOR_STATUS"]
    while time.monotonic() < deadline:
        msg = connection.recv_match(type=wanted, blocking=True, timeout=0.5)
        if msg is None:
            continue
        msg_type = msg.get_type()
        values: dict[str, Any] = {}
        if msg_type == "ATTITUDE":
            values = {
                "roll_deg": math.degrees(msg.roll),
                "pitch_deg": math.degrees(msg.pitch),
                "yaw_deg": math.degrees(msg.yaw),
                "rollspeed_deg_s": math.degrees(msg.rollspeed),
                "pitchspeed_deg_s": math.degrees(msg.pitchspeed),
                "yawspeed_deg_s": math.degrees(msg.yawspeed),
            }
        elif msg_type == "HIGHRES_IMU":
            values = {
                "xacc": msg.xacc,
                "yacc": msg.yacc,
                "zacc": msg.zacc,
                "xgyro": msg.xgyro,
                "ygyro": msg.ygyro,
                "zgyro": msg.zgyro,
            }
        elif msg_type == "RAW_IMU":
            values = {
                "xacc": msg.xacc,
                "yacc": msg.yacc,
                "zacc": msg.zacc,
                "xgyro": msg.xgyro,
                "ygyro": msg.ygyro,
                "zgyro": msg.zgyro,
            }
        elif msg_type == "RC_CHANNELS":
            values = {
                f"chan{i}_raw": getattr(msg, f"chan{i}_raw")
                for i in range(1, min(msg.chancount, 8) + 1)
                if getattr(msg, f"chan{i}_raw") not in (0, 65535)
            }
            values["rssi"] = msg.rssi
        elif msg_type == "ESTIMATOR_STATUS":
            values = {
                "flags": getattr(msg, "flags", None),
                "vel_ratio": getattr(msg, "vel_ratio", None),
                "pos_horiz_ratio": getattr(msg, "pos_horiz_ratio", None),
                "pos_vert_ratio": getattr(msg, "pos_vert_ratio", None),
                "hgt_ratio": getattr(msg, "hgt_ratio", None),
                "hagl_ratio": getattr(msg, "hagl_ratio", None),
                "tas_ratio": getattr(msg, "tas_ratio", None),
                "mag_ratio": getattr(msg, "mag_ratio", None),
            }
        samples.append(Sample(phase, time.time(), msg_type, values))
    return samples


def mean(values: list[float]) -> float | None:
    return statistics.fmean(values) if values else None


def stdev(values: list[float]) -> float | None:
    return statistics.pstdev(values) if len(values) > 1 else None


def summarize_phase(samples: list[Sample], phase: str) -> dict[str, Any]:
    phase_samples = [sample for sample in samples if sample.phase == phase]
    attitude = [sample for sample in phase_samples if sample.msg_type == "ATTITUDE"]
    rc = [sample for sample in phase_samples if sample.msg_type == "RC_CHANNELS"]
    highres_imu = [sample for sample in phase_samples if sample.msg_type == "HIGHRES_IMU"]
    raw_imu = [sample for sample in phase_samples if sample.msg_type == "RAW_IMU"]
    estimator = [
        sample for sample in phase_samples if sample.msg_type == "ESTIMATOR_STATUS"
    ]

    roll = [float(sample.values["roll_deg"]) for sample in attitude]
    pitch = [float(sample.values["pitch_deg"]) for sample in attitude]
    yaw = [float(sample.values["yaw_deg"]) for sample in attitude]

    rc_summary: dict[str, Any] = {}
    if rc:
        for channel in range(1, 9):
            key = f"chan{channel}_raw"
            channel_values = [
                float(sample.values[key]) for sample in rc if key in sample.values
            ]
            if channel_values:
                rc_summary[key] = {
                    "avg": mean(channel_values),
                    "min": min(channel_values),
                    "max": max(channel_values),
                }

    return {
        "phase": phase,
        "counts": {
            "ATTITUDE": len(attitude),
            "HIGHRES_IMU": len(highres_imu),
            "RAW_IMU": len(raw_imu),
            "RC_CHANNELS": len(rc),
            "ESTIMATOR_STATUS": len(estimator),
        },
        "attitude_deg": {
            "roll_avg": mean(roll),
            "roll_std": stdev(roll),
            "roll_min": min(roll) if roll else None,
            "roll_max": max(roll) if roll else None,
            "pitch_avg": mean(pitch),
            "pitch_std": stdev(pitch),
            "pitch_min": min(pitch) if pitch else None,
            "pitch_max": max(pitch) if pitch else None,
            "yaw_avg": mean(yaw),
            "yaw_std": stdev(yaw),
            "yaw_min": min(yaw) if yaw else None,
            "yaw_max": max(yaw) if yaw else None,
        },
        "rc_channels": rc_summary,
        "last_estimator_status": estimator[-1].values if estimator else None,
    }


def evaluate(
    phase_summaries: dict[str, dict[str, Any]], params: dict[str, float | None], threshold: float
) -> list[str]:
    findings: list[str] = []
    level = phase_summaries.get("level")
    if not level:
        return ["No level phase was captured, so attitude bias could not be evaluated."]

    attitude = level["attitude_deg"]
    pitch_avg = attitude["pitch_avg"]
    roll_avg = attitude["roll_avg"]
    if pitch_avg is None or roll_avg is None:
        findings.append("No ATTITUDE samples were captured.")
        return findings

    if abs(pitch_avg) > threshold:
        findings.append(
            f"Level pitch bias is {pitch_avg:+.3f} deg, above the {threshold:.3f} deg target."
        )
    else:
        findings.append(
            f"Level pitch bias is {pitch_avg:+.3f} deg, within the {threshold:.3f} deg target."
        )

    if abs(roll_avg) > threshold:
        findings.append(
            f"Level roll bias is {roll_avg:+.3f} deg, above the {threshold:.3f} deg target."
        )
    else:
        findings.append(
            f"Level roll bias is {roll_avg:+.3f} deg, within the {threshold:.3f} deg target."
        )

    sens_x = params.get("SENS_BOARD_X_OFF")
    sens_y = params.get("SENS_BOARD_Y_OFF")
    if sens_x is not None and sens_y is not None:
        findings.append(
            f"Current board offsets are X={sens_x:+.3f} deg, Y={sens_y:+.3f} deg."
        )

    rc = level.get("rc_channels") or {}
    for channel, trim_name in (("chan1_raw", "RC1_TRIM"), ("chan2_raw", "RC2_TRIM")):
        if channel not in rc or params.get(trim_name) is None:
            continue
        avg = rc[channel]["avg"]
        trim = params[trim_name]
        delta = avg - trim
        findings.append(f"{channel} avg is {avg:.1f}, {delta:+.1f} us from {trim_name}.")

    if "nose_down" in phase_summaries and "nose_up" in phase_summaries:
        down = phase_summaries["nose_down"]["attitude_deg"]["pitch_avg"]
        up = phase_summaries["nose_up"]["attitude_deg"]["pitch_avg"]
        if down is not None and up is not None:
            findings.append(
                f"Pitch sign check: nose_down={down:+.3f} deg, nose_up={up:+.3f} deg."
            )

    return findings


def write_samples_csv(path: Path, samples: list[Sample]) -> None:
    rows: list[dict[str, Any]] = []
    keys = {"phase", "timestamp_s", "msg_type"}
    for sample in samples:
        keys.update(sample.values.keys())
    ordered_keys = ["phase", "timestamp_s", "msg_type"] + sorted(
        key for key in keys if key not in {"phase", "timestamp_s", "msg_type"}
    )
    for sample in samples:
        row = {
            "phase": sample.phase,
            "timestamp_s": f"{sample.timestamp_s:.6f}",
            "msg_type": sample.msg_type,
        }
        row.update(sample.values)
        rows.append(row)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=ordered_keys)
        writer.writeheader()
        writer.writerows(rows)


def write_markdown_report(
    path: Path,
    endpoint: str,
    heartbeat: Any,
    params: dict[str, float | None],
    phase_summaries: dict[str, dict[str, Any]],
    findings: list[str],
) -> None:
    lines = [
        "# Stabilized Drift Bench Diagnostic",
        "",
        f"- Timestamp UTC: {datetime.now(timezone.utc).isoformat()}",
        f"- Endpoint: `{endpoint}`",
        f"- Heartbeat: sys={heartbeat.get_srcSystem()} comp={heartbeat.get_srcComponent()} type={heartbeat.type} autopilot={heartbeat.autopilot}",
        "- Safety: read-only MAVLink telemetry and PARAM_REQUEST_READ only; no arming and no parameter writes.",
        "",
        "## Findings",
        "",
    ]
    lines.extend(f"- {finding}" for finding in findings)
    lines.extend(["", "## Phase Summaries", ""])

    for name, summary in phase_summaries.items():
        attitude = summary["attitude_deg"]
        lines.extend(
            [
                f"### {name}",
                "",
                f"- Samples: {summary['counts']}",
                f"- Roll avg/std/min/max deg: {attitude['roll_avg']}, {attitude['roll_std']}, {attitude['roll_min']}, {attitude['roll_max']}",
                f"- Pitch avg/std/min/max deg: {attitude['pitch_avg']}, {attitude['pitch_std']}, {attitude['pitch_min']}, {attitude['pitch_max']}",
                "",
            ]
        )

    lines.extend(["## Parameters", ""])
    for key in sorted(params):
        lines.append(f"- `{key}`: {params[key]}")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def prompt_phase(phase: str) -> None:
    print()
    print(f"Prepare phase: {phase}")
    print("Keep ESCs disconnected. Press Enter when the airframe is held in this pose.")
    input()


def main() -> int:
    args = build_parser().parse_args()
    endpoints = args.endpoints or list(DEFAULT_ENDPOINTS)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    repo_root = Path(__file__).resolve().parents[1]
    output_dir = args.output_dir or repo_root / "temp_outputs" / f"stabilized_drift_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    connection, endpoint, heartbeat = connect(endpoints)
    print(
        f"Connected: endpoint={endpoint} sys={connection.target_system} "
        f"comp={connection.target_component}"
    )
    request_message_intervals(connection)

    print("Reading suspect params...")
    params = read_params(connection, args.param_timeout)

    phases = ["level"]
    if args.interactive_sign_test:
        phases = ["level", "nose_down", "nose_up", "roll_right", "roll_left"]

    samples: list[Sample] = []
    for phase in phases:
        if args.interactive_sign_test:
            prompt_phase(phase)
        print(f"Capturing {phase} for {args.duration:.1f}s...")
        samples.extend(capture_phase(connection, phase, args.duration))

    phase_summaries = {
        phase: summarize_phase(samples, phase)
        for phase in phases
    }
    findings = evaluate(phase_summaries, params, args.bias_threshold_deg)

    report = {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "endpoint": endpoint,
        "heartbeat": {
            "source_system": heartbeat.get_srcSystem(),
            "source_component": heartbeat.get_srcComponent(),
            "type": heartbeat.type,
            "autopilot": heartbeat.autopilot,
            "base_mode": heartbeat.base_mode,
            "custom_mode": heartbeat.custom_mode,
        },
        "params": params,
        "phase_summaries": phase_summaries,
        "findings": findings,
    }

    (output_dir / "summary.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_samples_csv(output_dir / "samples.csv", samples)
    write_markdown_report(
        output_dir / "summary.md",
        endpoint,
        heartbeat,
        params,
        phase_summaries,
        findings,
    )

    print()
    print("Findings:")
    for finding in findings:
        print(f"- {finding}")
    print()
    print(f"Saved report to {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
