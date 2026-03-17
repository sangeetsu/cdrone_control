from __future__ import annotations

from dataclasses import dataclass

import math
from typing import Tuple


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


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


@dataclass
class VelocityCommand:
    vx: float
    vy: float
    vz: float
    yaw_rate: float
    min_distance_gate_active: bool


def score_track(track: TrackSnapshot) -> float:
    p_norm = math.sqrt(track.x_b_m**2 + track.y_b_m**2 + track.z_b_m**2)
    radial_v = -(
        track.x_b_m * track.vx_b_mps
        + track.y_b_m * track.vy_b_mps
        + track.z_b_m * track.vz_b_mps
    ) / (p_norm + 1e-3)
    inbound_component = max(0.0, radial_v)
    distance_component = 1.0 / (track.distance_m + 0.1)
    return 0.7 * inbound_component + 0.3 * distance_component


def target_bearing_rad(track: TrackSnapshot) -> float:
    return math.atan2(track.y_b_m, max(track.x_b_m, 1e-3))


def compute_velocity_command(
    track: TrackSnapshot,
    desired_distance_m: float,
    kp_xy: float,
    kp_z: float,
    kp_yaw: float,
    max_vel_xy_mps: float,
    max_vel_z_mps: float,
    max_yaw_rate_rps: float,
    min_safe_distance_m: float,
) -> Tuple[float, float, float, float]:
    command = compute_velocity_command_details(
        track=track,
        desired_distance_m=desired_distance_m,
        kp_xy=kp_xy,
        kp_z=kp_z,
        kp_yaw=kp_yaw,
        max_vel_xy_mps=max_vel_xy_mps,
        max_vel_z_mps=max_vel_z_mps,
        max_yaw_rate_rps=max_yaw_rate_rps,
        min_safe_distance_m=min_safe_distance_m,
    )
    return command.vx, command.vy, command.vz, command.yaw_rate


def compute_velocity_command_details(
    track: TrackSnapshot,
    desired_distance_m: float,
    kp_xy: float,
    kp_z: float,
    kp_yaw: float,
    max_vel_xy_mps: float,
    max_vel_z_mps: float,
    max_yaw_rate_rps: float,
    min_safe_distance_m: float,
) -> VelocityCommand:
    err_forward = track.x_b_m - desired_distance_m
    err_lateral = track.y_b_m
    err_vertical = track.z_b_m
    yaw_error = target_bearing_rad(track)

    vx = clamp(kp_xy * err_forward, -max_vel_xy_mps, max_vel_xy_mps)
    vy = clamp(kp_xy * err_lateral, -max_vel_xy_mps, max_vel_xy_mps)
    vz = clamp(kp_z * err_vertical, -max_vel_z_mps, max_vel_z_mps)
    yaw_rate = clamp(kp_yaw * yaw_error, -max_yaw_rate_rps, max_yaw_rate_rps)

    gate_active = False
    if track.distance_m < min_safe_distance_m and vx > 0.0:
        vx = 0.0
        gate_active = True

    return VelocityCommand(
        vx=vx,
        vy=vy,
        vz=vz,
        yaw_rate=yaw_rate,
        min_distance_gate_active=gate_active,
    )
