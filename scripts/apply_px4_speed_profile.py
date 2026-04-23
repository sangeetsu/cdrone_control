#!/usr/bin/env python3
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = REPO_ROOT / "ros2" / "src" / "drone_bringup" / "config"
DEFAULT_PX4_SHELL = "/home/jetson/ARK-OS/platform/common/scripts/px4_shell_command.py"
DEFAULT_PX4_PORT = "udp:127.0.0.1:14540"
PROFILE_ALIASES = {
    "indoor": "indoor_speed_profile.yaml",
    "indoor_speed_profile": "indoor_speed_profile.yaml",
    "default": "default_speed_profile.yaml",
    "default_speed_profile": "default_speed_profile.yaml",
    "fun": "fun_speed_profile.yaml",
    "fun_speed_profile": "fun_speed_profile.yaml",
}


def resolve_profile_path(profile: str) -> Path:
    candidate = PROFILE_ALIASES.get(profile.strip(), profile.strip())
    path = Path(candidate).expanduser()
    if not path.is_absolute():
        path = CONFIG_DIR / path
    return path


def load_profile(path: Path) -> tuple[str, str, dict[str, float]]:
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
    return name, description, parameters


def build_commands(parameters: dict[str, float]) -> str:
    lines = [f"param set {param} {value}" for param, value in parameters.items()]
    lines.append("param save")
    lines.extend(f"param show {param}" for param in parameters)
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Apply a named PX4 speed profile to the flight controller."
    )
    parser.add_argument(
        "profile",
        nargs="?",
        default="indoor",
        help="Profile name or YAML path. Named presets: indoor, fun, default.",
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PX4_PORT,
        help=f"PX4 shell transport. Default: {DEFAULT_PX4_PORT}",
    )
    parser.add_argument(
        "--px4-shell",
        default=DEFAULT_PX4_SHELL,
        help=f"Path to px4_shell_command.py. Default: {DEFAULT_PX4_SHELL}",
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
        for alias in ("indoor", "fun", "default"):
            print(f"{alias}: {CONFIG_DIR / PROFILE_ALIASES[alias]}")
        return 0

    profile_path = resolve_profile_path(args.profile)
    profile_name, description, parameters = load_profile(profile_path)
    commands = build_commands(parameters)

    print(f"Profile: {profile_name}")
    print(f"Source:  {profile_path}")
    if description:
        print(f"About:   {description}")
    print("Params:")
    for name, value in parameters.items():
        print(f"  {name} = {value}")

    if args.dry_run:
        print("\nCommands:\n")
        print(commands)
        return 0

    px4_shell_path = Path(args.px4_shell).expanduser()
    if not px4_shell_path.is_file():
        raise FileNotFoundError(f"px4 shell helper not found: {px4_shell_path}")

    subprocess.run(
        [sys.executable, str(px4_shell_path), "--port", args.port, commands],
        check=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
