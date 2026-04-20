from __future__ import annotations

import math
from dataclasses import dataclass


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) <= max(deadband, 0.0) else value


@dataclass
class TrackSnapshot:
    track_id: int
    x_b_m: float
    y_b_m: float
    z_b_m: float
    vx_b_mps: float
    vy_b_mps: float
    vz_b_mps: float
    distance_m: float
    confidence: float
    bbox_area_px: float
    inbound: bool
    last_seen_s: float


@dataclass(frozen=True)
class FollowCommand:
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    err_forward: float
    err_lateral: float
    err_vertical: float
    yaw_error: float
    min_distance_gate_active: bool


def target_bearing_rad(track: TrackSnapshot) -> float:
    return math.atan2(track.y_b_m, max(track.x_b_m, 1e-3))


def score_track(track: TrackSnapshot) -> float:
    position_norm = math.sqrt(
        track.x_b_m * track.x_b_m
        + track.y_b_m * track.y_b_m
        + track.z_b_m * track.z_b_m
    )
    radial_v = -(
        track.x_b_m * track.vx_b_mps
        + track.y_b_m * track.vy_b_mps
        + track.z_b_m * track.vz_b_mps
    ) / (position_norm + 1e-3)
    inbound_component = max(0.0, radial_v)
    distance_component = 1.0 / (track.distance_m + 0.1)
    confidence_component = max(track.confidence, 0.0)
    return (
        0.55 * inbound_component
        + 0.30 * distance_component
        + 0.15 * confidence_component
    )


def track_is_valid(
    track: TrackSnapshot,
    *,
    min_track_confidence: float,
    max_target_distance_m: float,
    require_target_in_front: bool,
    max_abs_target_y_m: float,
    max_abs_target_z_m: float,
) -> bool:
    if track.confidence < min_track_confidence:
        return False
    if track.distance_m > max_target_distance_m:
        return False
    if require_target_in_front and track.x_b_m <= 0.0:
        return False
    if abs(track.y_b_m) > max_abs_target_y_m:
        return False
    if abs(track.z_b_m) > max_abs_target_z_m:
        return False
    return True


def distance_in_standoff_window(
    distance_m: float,
    desired_distance_m: float,
    tolerance_m: float,
) -> bool:
    return abs(float(distance_m) - float(desired_distance_m)) <= max(
        float(tolerance_m), 0.0
    )


def compute_follow_command(
    track: TrackSnapshot,
    *,
    follow_distance_m: float,
    follow_distance_tolerance_m: float,
    lateral_deadband_m: float,
    vertical_deadband_m: float,
    yaw_deadband_rad: float,
    kp_xy: float,
    kp_z: float,
    kp_yaw: float,
    max_vel_xy_mps: float,
    max_vel_z_mps: float,
    max_yaw_rate_rps: float,
    min_safe_distance_m: float,
) -> FollowCommand:
    err_forward = apply_deadband(
        track.x_b_m - follow_distance_m,
        follow_distance_tolerance_m,
    )
    err_lateral = apply_deadband(track.y_b_m, lateral_deadband_m)
    err_vertical = apply_deadband(track.z_b_m, vertical_deadband_m)
    yaw_error = apply_deadband(target_bearing_rad(track), yaw_deadband_rad)

    vx = clamp(kp_xy * err_forward, -max_vel_xy_mps, max_vel_xy_mps)
    vy = clamp(kp_xy * err_lateral, -max_vel_xy_mps, max_vel_xy_mps)
    vz = clamp(kp_z * err_vertical, -max_vel_z_mps, max_vel_z_mps)
    yaw_rate = clamp(
        kp_yaw * yaw_error,
        -max_yaw_rate_rps,
        max_yaw_rate_rps,
    )

    min_distance_gate_active = False
    if track.distance_m < min_safe_distance_m and vx > 0.0:
        vx = 0.0
        min_distance_gate_active = True

    return FollowCommand(
        vx=vx,
        vy=vy,
        vz=vz,
        yaw_rate=yaw_rate,
        err_forward=err_forward,
        err_lateral=err_lateral,
        err_vertical=err_vertical,
        yaw_error=yaw_error,
        min_distance_gate_active=min_distance_gate_active,
    )
