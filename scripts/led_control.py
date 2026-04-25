#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:  # pragma: no cover - optional outside the repo env
    yaml = None


DEFAULT_ENGAGEMENT_STATE_TOPIC = "/cdrone/cdrone3/engagement/state"


@dataclass(frozen=True)
class Color:
    name: str
    r: int
    g: int
    b: int

    @property
    def rgb(self) -> tuple[int, int, int]:
        return (self.r, self.g, self.b)


COLORS: dict[str, Color] = {
    "off": Color("off", 0, 0, 0),
    "startup": Color("startup", 0, 80, 255),
    "search": Color("search", 255, 220, 0),
    "follow": Color("follow", 0, 255, 0),
    "dwell": Color("dwell", 0, 255, 255),
    "return": Color("return", 180, 0, 255),
    "landing": Color("landing", 255, 120, 0),
    "complete": Color("complete", 255, 255, 255),
    "blocked": Color("blocked", 255, 60, 0),
    "abort": Color("abort", 255, 0, 0),
    "unknown": Color("unknown", 120, 120, 120),
}

TEST_COLORS: dict[str, Color] = {
    **COLORS,
    "blue": COLORS["startup"],
    "yellow": COLORS["search"],
    "green": COLORS["follow"],
    "cyan": COLORS["dwell"],
    "violet": COLORS["return"],
    "purple": COLORS["return"],
    "amber": COLORS["landing"],
    "orange": COLORS["blocked"],
    "red": COLORS["abort"],
    "white": COLORS["complete"],
}

STARTUP_STATES = {
    "SYNC_TAKEOFF_PARAM",
    "SYNC_PRE_TAKEOFF_PROFILE",
    "SET_TAKEOFF_MODE",
    "ARMING",
    "REQUEST_TAKEOFF",
    "TAKEOFF",
    "WARMUP_OFFBOARD",
    "SET_OFFBOARD_MODE",
    "SYNC_SPEED_PROFILE",
    "STAGE_HOVER",
}

RETURN_STATES = {
    "RETURN_TO_START",
    "RETURN_HOME_HOLD",
}

LANDING_STATES = {
    "SET_LAND_MODE",
    "WAIT_TOUCHDOWN",
    "DISARMING",
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def default_state_topic() -> str:
    env_topic = (
        os.environ.get("ENGAGEMENT_STATE_TOPIC", "").strip()
        or os.environ.get("MILESTONE3_STATE_TOPIC", "").strip()
    )
    if env_topic:
        return env_topic

    config_path = repo_root() / "ros2/src/drone_bringup/config/droneid_config.yaml"
    if yaml is None or not config_path.is_file():
        return DEFAULT_ENGAGEMENT_STATE_TOPIC

    try:
        config = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return DEFAULT_ENGAGEMENT_STATE_TOPIC

    if not isinstance(config, dict):
        return DEFAULT_ENGAGEMENT_STATE_TOPIC

    identity = config.get("identity", {})
    if not isinstance(identity, dict):
        identity = {}

    drone_id = str(identity.get("drone_id") or config.get("drone_id") or "").strip("/")
    if drone_id:
        return f"/cdrone/{drone_id}/engagement/state"

    return DEFAULT_ENGAGEMENT_STATE_TOPIC


def color_for_engagement_state(
    *, state: str, blocked_reason: str = "", estop_latched: bool = False
) -> Color:
    state_upper = str(state or "").strip().upper()
    blocked = bool(str(blocked_reason or "").strip())

    if estop_latched or state_upper == "ABORT":
        return COLORS["abort"]
    if blocked and state_upper not in {"IDLE", "COMPLETE"}:
        return COLORS["blocked"]
    if state_upper == "IDLE":
        return COLORS["off"]
    if state_upper == "COMPLETE":
        return COLORS["complete"]
    if state_upper in STARTUP_STATES:
        return COLORS["startup"]
    if state_upper == "SEARCH":
        return COLORS["search"]
    if state_upper == "FOLLOW":
        return COLORS["follow"]
    if state_upper == "DWELL":
        return COLORS["dwell"]
    if state_upper in RETURN_STATES:
        return COLORS["return"]
    if state_upper in LANDING_STATES:
        return COLORS["landing"]

    return COLORS["unknown"]


class DryRunLight:
    def set_color(self, color: Color) -> None:
        print(f"[dry-run] blink(1) -> {color.name} rgb={color.rgb}", flush=True)

    def close(self, *, turn_off: bool = True) -> None:
        if turn_off:
            self.set_color(COLORS["off"])


class Blink1Light:
    def __init__(self, fade_ms: int) -> None:
        from blink1.blink1 import Blink1

        self._blink = Blink1()
        self._fade_ms = int(fade_ms)

    def set_color(self, color: Color) -> None:
        self._blink.fade_to_rgb(self._fade_ms, color.r, color.g, color.b)

    def close(self, *, turn_off: bool = True) -> None:
        if turn_off:
            self.set_color(COLORS["off"])
        self._blink.close()


def make_light(*, dry_run: bool, fade_ms: int) -> DryRunLight | Blink1Light:
    if dry_run:
        return DryRunLight()
    try:
        return Blink1Light(fade_ms)
    except Exception as exc:
        raise RuntimeError(blink1_error_help(exc)) from exc


def blink1_error_help(exc: Exception) -> str:
    install_script = repo_root() / "scripts/install_blink1_host_access.sh"
    return (
        f"Could not open blink(1): {exc}\n\n"
        "The blink(1) USB HID device usually needs a udev rule before a normal "
        "user can open it for writing. Install the repo rule once, then unplug "
        "and replug the blink(1):\n"
        f"  sudo {install_script}\n"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Drive a blink(1) LED from drone_msgs/msg/EngagementState.state."
        )
    )
    parser.add_argument(
        "--state-topic",
        default=default_state_topic(),
        help="Milestone 3 EngagementState topic to subscribe to.",
    )
    parser.add_argument(
        "--fade-ms",
        type=int,
        default=250,
        help="blink(1) fade time in milliseconds.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Log LED changes without opening the blink(1) device.",
    )
    parser.add_argument(
        "--test-color",
        choices=sorted(TEST_COLORS),
        help="Set one color and exit without starting ROS.",
    )
    return parser.parse_args()


def run_test_color(args: argparse.Namespace) -> int:
    light = make_light(dry_run=bool(args.dry_run), fade_ms=int(args.fade_ms))
    try:
        light.set_color(TEST_COLORS[args.test_color])
    finally:
        light.close(turn_off=False)
    return 0


def run_ros(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from drone_msgs.msg import EngagementState
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
    except Exception as exc:
        print(
            "Could not import ROS 2 drone message modules. Source the ROS "
            "environment first, for example:\n"
            "  source /opt/ros/humble/setup.bash\n"
            "  source ros2/install/setup.bash\n\n"
            f"Import error: {exc}",
            file=sys.stderr,
        )
        return 2

    light = make_light(dry_run=bool(args.dry_run), fade_ms=int(args.fade_ms))

    class Milestone3StateLedNode(Node):
        def __init__(self) -> None:
            super().__init__("milestone3_state_led")
            self._last_color_name = ""
            self._last_log_key: tuple[Any, ...] | None = None
            self._sub = self.create_subscription(
                EngagementState, args.state_topic, self._state_callback, 10
            )
            self._apply_color(
                COLORS["startup"],
                ("waiting",),
                "waiting for milestone 3 engagement state messages",
            )
            self.get_logger().info(f"Subscribed to {args.state_topic}")

        def _state_callback(self, msg: Any) -> None:
            state = str(msg.state or "").strip().upper()
            blocked_reason = str(msg.blocked_reason or "").strip()
            color = color_for_engagement_state(
                state=state,
                blocked_reason=blocked_reason,
                estop_latched=bool(msg.estop_latched),
            )
            log_key = (
                color.name,
                state,
                blocked_reason,
                bool(msg.estop_latched),
                int(msg.active_track_id),
                bool(msg.target_visible),
            )
            summary = (
                f"state={state or '<empty>'} ready={bool(msg.autonomy_ready)} "
                f"target_visible={bool(msg.target_visible)} "
                f"active_track_id={int(msg.active_track_id)} "
                f"blocked_reason={blocked_reason or '<none>'}"
            )
            self._apply_color(color, log_key, summary)

        def _apply_color(
            self, color: Color, log_key: tuple[Any, ...], summary: str
        ) -> None:
            if color.name != self._last_color_name:
                self._last_color_name = color.name
                light.set_color(color)
            if log_key != self._last_log_key:
                self._last_log_key = log_key
                self.get_logger().info(f"LED color {color.name}: {summary}")

    rclpy.init()
    node = Milestone3StateLedNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        light.close(turn_off=True)
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main() -> int:
    args = parse_args()
    try:
        if args.test_color:
            return run_test_color(args)
        return run_ros(args)
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
