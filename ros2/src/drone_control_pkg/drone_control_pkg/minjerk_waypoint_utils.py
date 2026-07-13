from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Optional

import yaml


def angle_diff_rad(target_rad: float, source_rad: float) -> float:
    delta = float(target_rad) - float(source_rad)
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


def yaw_rad_from_deg(value: object) -> float:
    return math.radians(float(value))


def yaw_rad_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    return math.atan2(
        2.0 * ((w * z) + (x * y)),
        1.0 - (2.0 * ((y * y) + (z * z))),
    )


def minimum_jerk_blend(progress: float) -> float:
    t = min(max(float(progress), 0.0), 1.0)
    return (10.0 * t**3) - (15.0 * t**4) + (6.0 * t**5)


@dataclass(frozen=True)
class TrajectoryPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Waypoint:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: Optional[float]
    hold_s: Optional[float]
    duration_s: Optional[float]
    name: str = ""


@dataclass(frozen=True)
class WaypointRouteConfig:
    start_policy: str = "fixed"
    loop: bool = False
    return_to_start: bool = False


@dataclass(frozen=True)
class WaypointMission:
    name: str
    frame_id: str
    waypoints: tuple[Waypoint, ...]
    route: WaypointRouteConfig
    cruise_speed_mps: float
    min_segment_duration_s: float
    default_hold_s: float
    final_hold_s: float


@dataclass(frozen=True)
class MinJerkSegment:
    start: TrajectoryPose
    end: TrajectoryPose
    duration_s: float
    hold_s: float
    waypoint_name: str = ""


WaypointReachability = Callable[[TrajectoryPose, Waypoint], Optional[str]]


def load_waypoint_mission(path: str) -> WaypointMission:
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FileNotFoundError(f"waypoints config not found: {config_path}")

    loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"expected mapping in waypoints config: {config_path}")

    defaults = loaded.get("defaults") or {}
    if not isinstance(defaults, dict):
        raise ValueError("defaults must be a mapping")

    waypoints_raw = loaded.get("waypoints")
    if not isinstance(waypoints_raw, list) or not waypoints_raw:
        raise ValueError("waypoints must be a non-empty list")

    default_hold_s = _float_default(defaults, "hold_s", 0.0)
    waypoints = tuple(
        _parse_waypoint(item, default_hold_s=default_hold_s)
        for item in waypoints_raw
    )

    return WaypointMission(
        name=str(loaded.get("name", "")).strip() or config_path.stem,
        frame_id=str(loaded.get("frame_id", "")).strip() or "map",
        waypoints=waypoints,
        route=_parse_route_config(loaded.get("route")),
        cruise_speed_mps=max(
            _float_default(defaults, "cruise_speed_mps", 0.35),
            1e-3,
        ),
        min_segment_duration_s=max(
            _float_default(defaults, "min_segment_duration_s", 2.0),
            1e-3,
        ),
        default_hold_s=max(default_hold_s, 0.0),
        final_hold_s=max(_float_default(defaults, "final_hold_s", 2.0), 0.0),
    )


def build_minjerk_segments(
    start_pose: TrajectoryPose,
    mission: WaypointMission,
    *,
    waypoints: Optional[tuple[Waypoint, ...]] = None,
) -> tuple[MinJerkSegment, ...]:
    segments: list[MinJerkSegment] = []
    previous = start_pose
    previous_yaw = start_pose.yaw_rad
    mission_waypoints = mission.waypoints if waypoints is None else waypoints

    for index, waypoint in enumerate(mission_waypoints):
        target_yaw = (
            previous_yaw
            if waypoint.yaw_rad is None
            else previous_yaw + angle_diff_rad(waypoint.yaw_rad, previous_yaw)
        )
        target = TrajectoryPose(
            x_m=waypoint.x_m,
            y_m=waypoint.y_m,
            z_m=waypoint.z_m,
            yaw_rad=target_yaw,
        )
        duration_s = waypoint.duration_s
        if duration_s is None:
            duration_s = segment_duration_from_speed(
                previous,
                target,
                cruise_speed_mps=mission.cruise_speed_mps,
                min_segment_duration_s=mission.min_segment_duration_s,
            )
        hold_s = waypoint.hold_s
        if hold_s is None:
            hold_s = mission.default_hold_s
        if index == len(mission_waypoints) - 1:
            hold_s = max(float(hold_s), mission.final_hold_s)
        segments.append(
            MinJerkSegment(
                start=previous,
                end=target,
                duration_s=max(float(duration_s), 1e-3),
                hold_s=max(float(hold_s), 0.0),
                waypoint_name=waypoint.name,
            )
        )
        previous = target
        previous_yaw = target.yaw_rad

    return tuple(segments)


def resolve_waypoints_for_start(
    start_pose: TrajectoryPose,
    mission: WaypointMission,
    *,
    waypoint_reachable: Optional[WaypointReachability] = None,
    return_pose: Optional[TrajectoryPose] = None,
) -> tuple[Waypoint, ...]:
    ordered_waypoints = mission.waypoints
    route = mission.route

    if route.start_policy == "nearest_reachable":
        if waypoint_reachable is None:
            raise ValueError(
                "nearest_reachable waypoint route requires a reachability checker"
            )
        candidates: list[tuple[float, int]] = []
        for index, waypoint in enumerate(mission.waypoints):
            reason = waypoint_reachable(start_pose, waypoint)
            if reason is not None:
                continue
            distance_m = math.hypot(
                waypoint.x_m - start_pose.x_m,
                waypoint.y_m - start_pose.y_m,
            )
            candidates.append((distance_m, index))
        if not candidates:
            raise ValueError("no reachable waypoint entry from start position")
        _, entry_index = min(candidates, key=lambda item: (item[0], item[1]))
        ordered_waypoints = (
            mission.waypoints[entry_index:] + mission.waypoints[:entry_index]
        )
    elif route.start_policy != "fixed":
        raise ValueError(
            "route.start_policy must be one of: fixed, nearest_reachable"
        )

    resolved_waypoints = tuple(ordered_waypoints)
    if route.loop and resolved_waypoints:
        resolved_waypoints = resolved_waypoints + (resolved_waypoints[0],)
    if route.return_to_start:
        route_return_pose = start_pose if return_pose is None else return_pose
        return_z_m = (
            resolved_waypoints[-1].z_m if resolved_waypoints else start_pose.z_m
        )
        resolved_waypoints = resolved_waypoints + (
            Waypoint(
                x_m=route_return_pose.x_m,
                y_m=route_return_pose.y_m,
                z_m=return_z_m,
                yaw_rad=None,
                hold_s=None,
                duration_s=None,
                name="return_to_start",
            ),
        )
    return resolved_waypoints


def sample_minjerk_segment(
    segment: MinJerkSegment,
    elapsed_s: float,
) -> TrajectoryPose:
    progress = minimum_jerk_blend(float(elapsed_s) / segment.duration_s)
    return interpolate_pose(segment.start, segment.end, progress)


def segment_duration_from_speed(
    start: TrajectoryPose,
    end: TrajectoryPose,
    *,
    cruise_speed_mps: float,
    min_segment_duration_s: float,
) -> float:
    distance_m = math.sqrt(
        ((end.x_m - start.x_m) ** 2)
        + ((end.y_m - start.y_m) ** 2)
        + ((end.z_m - start.z_m) ** 2)
    )
    return max(float(min_segment_duration_s), distance_m / cruise_speed_mps)


def interpolate_pose(
    start: TrajectoryPose,
    end: TrajectoryPose,
    blend: float,
) -> TrajectoryPose:
    t = min(max(float(blend), 0.0), 1.0)
    return TrajectoryPose(
        x_m=start.x_m + ((end.x_m - start.x_m) * t),
        y_m=start.y_m + ((end.y_m - start.y_m) * t),
        z_m=start.z_m + ((end.z_m - start.z_m) * t),
        yaw_rad=start.yaw_rad + ((end.yaw_rad - start.yaw_rad) * t),
    )


def _parse_waypoint(
    raw: object,
    *,
    default_hold_s: float,
) -> Waypoint:
    if not isinstance(raw, dict):
        raise ValueError("each waypoint must be a mapping")
    yaw_rad = None
    if "yaw_rad" in raw:
        yaw_rad = float(raw["yaw_rad"])
    elif "yaw_deg" in raw:
        yaw_rad = yaw_rad_from_deg(raw["yaw_deg"])
    return Waypoint(
        x_m=float(raw["x_m"]),
        y_m=float(raw["y_m"]),
        z_m=float(raw["z_m"]),
        yaw_rad=yaw_rad,
        hold_s=_optional_float(raw.get("hold_s", default_hold_s)),
        duration_s=_optional_float(raw.get("duration_s")),
        name=str(raw.get("name", "")).strip(),
    )


def _parse_route_config(raw: object) -> WaypointRouteConfig:
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("route must be a mapping")

    start_policy = str(raw.get("start_policy", "fixed")).strip() or "fixed"
    if start_policy not in {"fixed", "nearest_reachable"}:
        raise ValueError(
            "route.start_policy must be one of: fixed, nearest_reachable"
        )

    return WaypointRouteConfig(
        start_policy=start_policy,
        loop=_bool_default(raw, "loop", False),
        return_to_start=_bool_default(raw, "return_to_start", False),
    )


def _float_default(
    mapping: dict[str, Any],
    key: str,
    fallback: float,
) -> float:
    value = mapping.get(key, fallback)
    if value is None:
        return float(fallback)
    return float(value)


def _bool_default(
    mapping: dict[str, Any],
    key: str,
    fallback: bool,
) -> bool:
    value = mapping.get(key, fallback)
    if value is None:
        return bool(fallback)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    raise ValueError(f"{key} must be a boolean")


def _optional_float(value: object) -> Optional[float]:
    if value is None:
        return None
    return float(value)
