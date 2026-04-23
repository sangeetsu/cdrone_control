#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from pymavlink import mavutil
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO_ROOT / "ros2" / "src" / "drone_bringup" / "config"
DEFAULT_PX4_PORT = "udpout:127.0.0.1:14550"
DEFAULT_READ_TIMEOUT_S = 1.5
DEFAULT_RETRIES = 3
DEFAULT_TARGET_SYSTEM = 1
DEFAULT_TARGET_COMPONENT = 1
DEFAULT_SOURCE_SYSTEM = 250
DEFAULT_SOURCE_COMPONENT = 190
DEFAULT_TOLERANCE = 1e-3
PROFILE_ALIASES = {
    "indoor": "indoor_speed_profile.yaml",
    "indoor_speed_profile": "indoor_speed_profile.yaml",
    "default": "default_speed_profile.yaml",
    "default_speed_profile": "default_speed_profile.yaml",
    "fun": "fun_speed_profile.yaml",
    "fun_speed_profile": "fun_speed_profile.yaml",
    "baseline": "hover_baseline_profile.yaml",
    "hover_baseline": "hover_baseline_profile.yaml",
    "hover_baseline_profile": "hover_baseline_profile.yaml",
    "takeoff_baseline": "milestone3_takeoff_baseline_profile.yaml",
    "milestone3_takeoff_baseline": "milestone3_takeoff_baseline_profile.yaml",
}


@dataclass(frozen=True)
class Profile:
    name: str
    description: str
    parameters: dict[str, float]
    source_path: Path


def resolve_profile_path(profile: str) -> Path:
    candidate = PROFILE_ALIASES.get(profile.strip(), profile.strip())
    path = Path(candidate).expanduser()
    if not path.is_absolute():
        path = CONFIG_DIR / path
    return path


def load_profile(path: Path) -> Profile:
    if not path.is_file():
        raise FileNotFoundError(f"profile not found: {path}")

    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"expected mapping in profile: {path}")

    raw_parameters = loaded.get("parameters")
    if not isinstance(raw_parameters, dict) or not raw_parameters:
        raise ValueError(f"profile must define a non-empty parameters mapping: {path}")

    parameters: dict[str, float] = {}
    for raw_name, raw_value in raw_parameters.items():
        name = str(raw_name or "").strip()
        if not name:
            raise ValueError(f"empty parameter name in profile: {path}")
        parameters[name] = float(raw_value)

    name = str(loaded.get("name", "")).strip() or path.stem
    description = str(loaded.get("description", "")).strip()
    return Profile(
        name=name,
        description=description,
        parameters=parameters,
        source_path=path,
    )


def build_commands(parameters: dict[str, float]) -> str:
    lines = [f"param set {param} {value}" for param, value in parameters.items()]
    lines.append("param save")
    lines.extend(f"param show {param}" for param in parameters)
    return "\n".join(lines)


def encode_param_id(name: str) -> bytes:
    return str(name).encode("ascii", errors="strict")[:16].ljust(16, b"\x00")


def decode_param_id(raw: object) -> str:
    if isinstance(raw, (bytes, bytearray)):
        return bytes(raw).decode("ascii", errors="ignore").rstrip("\x00")
    return str(raw or "").rstrip("\x00")


def candidate_ports(explicit_port: str) -> list[str]:
    requested = str(explicit_port or "").strip()
    env_port = str(os.environ.get("PX4_PORT", "")).strip()
    candidates: list[str] = []

    def add(port: str) -> None:
        normalized = str(port or "").strip()
        if normalized and normalized not in candidates:
            candidates.append(normalized)

    if requested:
        add(requested)
    elif env_port:
        add(env_port)

    for candidate in list(candidates):
        if candidate.startswith("udp:") and "@" not in candidate:
            add("udpout:" + candidate[len("udp:"):])

    add(DEFAULT_PX4_PORT)
    return candidates


def prime_router_route(connection: object) -> None:
    connection.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def wait_for_param_value(
    connection: object,
    param_name: str,
    *,
    timeout_s: float,
) -> object | None:
    deadline = time.monotonic() + max(timeout_s, 0.0)
    while True:
        remaining_s = deadline - time.monotonic()
        if remaining_s <= 0.0:
            return None
        msg = connection.recv_match(
            type="PARAM_VALUE",
            blocking=True,
            timeout=min(remaining_s, 0.5),
        )
        if msg is None:
            continue
        if decode_param_id(msg.param_id) == param_name:
            return msg


def request_param_value(
    connection: object,
    param_name: str,
    *,
    target_system: int,
    target_component: int,
    timeout_s: float,
    retries: int,
) -> float:
    encoded_name = encode_param_id(param_name)
    for _ in range(max(retries, 1)):
        prime_router_route(connection)
        connection.mav.param_request_read_send(
            int(target_system),
            int(target_component),
            encoded_name,
            -1,
        )
        msg = wait_for_param_value(connection, param_name, timeout_s=timeout_s)
        if msg is not None:
            return float(msg.param_value)
    raise TimeoutError(f"timed out waiting for {param_name} on the MAVLink parameter link")


def set_param_value(
    connection: object,
    param_name: str,
    desired_value: float,
    *,
    target_system: int,
    target_component: int,
    timeout_s: float,
    retries: int,
    tolerance: float,
) -> float:
    encoded_name = encode_param_id(param_name)
    for _ in range(max(retries, 1)):
        prime_router_route(connection)
        connection.mav.param_set_send(
            int(target_system),
            int(target_component),
            encoded_name,
            float(desired_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        msg = wait_for_param_value(connection, param_name, timeout_s=timeout_s)
        if msg is None:
            continue
        acknowledged_value = float(msg.param_value)
        if abs(acknowledged_value - float(desired_value)) <= max(tolerance, 0.0):
            return acknowledged_value
    raise TimeoutError(
        f"timed out waiting for a matching PARAM_VALUE ack for {param_name}={float(desired_value):.6g}"
    )


def connect_and_probe(
    ports: list[str],
    probe_param_name: str,
    *,
    source_system: int,
    source_component: int,
    target_system: int,
    target_component: int,
    timeout_s: float,
    retries: int,
) -> tuple[object, str]:
    errors: list[str] = []
    for port in ports:
        connection = None
        try:
            connection = mavutil.mavlink_connection(
                port,
                source_system=int(source_system),
                source_component=int(source_component),
                autoreconnect=False,
            )
            request_param_value(
                connection,
                probe_param_name,
                target_system=target_system,
                target_component=target_component,
                timeout_s=timeout_s,
                retries=retries,
            )
            return connection, port
        except Exception as exc:
            errors.append(f"{port}: {exc}")
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
    joined = "\n".join(f"  - {error}" for error in errors) or "  - no candidate ports tried"
    raise RuntimeError(
        "Unable to reach PX4 over MAVLink parameter transport.\n"
        "Tried:\n"
        f"{joined}\n"
        f"Expected mavlink-router on {DEFAULT_PX4_PORT}."
    )


def format_value(value: float) -> str:
    return f"{float(value):.6g}"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply a named PX4 speed profile directly over MAVLink PARAM_SET."
    )
    parser.add_argument(
        "profile",
        nargs="?",
        default="indoor",
        help="Profile name or YAML path. Named presets: indoor, fun, default, baseline.",
    )
    parser.add_argument(
        "--port",
        default="",
        help=(
            "Preferred MAVLink endpoint. If omitted, the script honors PX4_PORT when set "
            f"and otherwise uses {DEFAULT_PX4_PORT}."
        ),
    )
    parser.add_argument(
        "--read-timeout-s",
        type=float,
        default=DEFAULT_READ_TIMEOUT_S,
        help=f"Timeout per PARAM_VALUE wait. Default: {DEFAULT_READ_TIMEOUT_S}",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=DEFAULT_RETRIES,
        help=f"How many times to retry each read/write. Default: {DEFAULT_RETRIES}",
    )
    parser.add_argument(
        "--target-system",
        type=int,
        default=DEFAULT_TARGET_SYSTEM,
        help=f"MAVLink target system ID. Default: {DEFAULT_TARGET_SYSTEM}",
    )
    parser.add_argument(
        "--target-component",
        type=int,
        default=DEFAULT_TARGET_COMPONENT,
        help=f"MAVLink target component ID. Default: {DEFAULT_TARGET_COMPONENT}",
    )
    parser.add_argument(
        "--source-system",
        type=int,
        default=DEFAULT_SOURCE_SYSTEM,
        help=f"MAVLink source system ID. Default: {DEFAULT_SOURCE_SYSTEM}",
    )
    parser.add_argument(
        "--source-component",
        type=int,
        default=DEFAULT_SOURCE_COMPONENT,
        help=f"MAVLink source component ID. Default: {DEFAULT_SOURCE_COMPONENT}",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE,
        help=f"Acceptable floating-point ack error. Default: {DEFAULT_TOLERANCE}",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the resolved commands without sending them to PX4.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List the built-in profile aliases and exit.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list:
        for alias in (
            "indoor",
            "fun",
            "default",
            "baseline",
            "takeoff_baseline",
        ):
            print(f"{alias}: {CONFIG_DIR / PROFILE_ALIASES[alias]}")
        return 0

    profile_path = resolve_profile_path(args.profile)
    profile = load_profile(profile_path)
    commands = build_commands(profile.parameters)
    ports = candidate_ports(args.port)

    print(f"Profile: {profile.name}")
    print(f"Source:  {profile.source_path}")
    if profile.description:
        print(f"About:   {profile.description}")
    print("Params:")
    for name, value in profile.parameters.items():
        print(f"  {name} = {format_value(value)}")

    if args.dry_run:
        print("\nPort candidates:")
        for port in ports:
            print(f"  {port}")
        print("\nCommands:\n")
        print(commands)
        return 0

    connection = None
    try:
        first_param_name = next(iter(profile.parameters))
        connection, active_port = connect_and_probe(
            ports,
            first_param_name,
            source_system=args.source_system,
            source_component=args.source_component,
            target_system=args.target_system,
            target_component=args.target_component,
            timeout_s=args.read_timeout_s,
            retries=args.retries,
        )
        print(f"Endpoint: {active_port}")
        print("Applying:")

        changed_count = 0
        skipped_count = 0
        cached_values: dict[str, float] = {}

        for param_name, desired_value in profile.parameters.items():
            current_value = cached_values.get(param_name)
            if current_value is None:
                current_value = request_param_value(
                    connection,
                    param_name,
                    target_system=args.target_system,
                    target_component=args.target_component,
                    timeout_s=args.read_timeout_s,
                    retries=args.retries,
                )
                cached_values[param_name] = current_value

            if abs(current_value - desired_value) <= max(args.tolerance, 0.0):
                skipped_count += 1
                print(
                    f"  {param_name}: already {format_value(current_value)}; skipped"
                )
                continue

            acknowledged_value = set_param_value(
                connection,
                param_name,
                desired_value,
                target_system=args.target_system,
                target_component=args.target_component,
                timeout_s=args.read_timeout_s,
                retries=args.retries,
                tolerance=args.tolerance,
            )
            changed_count += 1
            print(
                "  "
                f"{param_name}: {format_value(current_value)} -> {format_value(acknowledged_value)}"
            )

        print(
            f"Done. changed={changed_count} skipped={skipped_count} total={len(profile.parameters)}"
        )
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
