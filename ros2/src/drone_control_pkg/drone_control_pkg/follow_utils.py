from __future__ import annotations

import math
from dataclasses import dataclass, replace


TRACK_SOURCE_DETECTED = 0
TRACK_SOURCE_HELD = 1
TRACK_SOURCE_PREDICTED = 2


def clamp(value: float, min_v: float, max_v: float) -> float:
    return max(min(value, max_v), min_v)


def apply_deadband(value: float, deadband: float) -> float:
    return 0.0 if abs(value) <= max(deadband, 0.0) else value


def wrap_angle_rad(angle_rad: float) -> float:
    return math.atan2(math.sin(float(angle_rad)), math.cos(float(angle_rad)))


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
    detector_track_id: int = -1
    source: int = TRACK_SOURCE_DETECTED
    last_observed_age_s: float = 0.0
    prediction_horizon_s: float = 0.0
    position_uncertainty_m: float = 0.0
    velocity_uncertainty_mps: float = 0.0


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


def clamp_follow_command_altitude(
    command: FollowCommand,
    *,
    current_altitude_m: float,
    projected_horizon_s: float,
    min_z_m: float | None,
    max_z_m: float | None,
) -> FollowCommand:
    horizon_s = max(float(projected_horizon_s), 0.0)
    if horizon_s <= 1e-6:
        return command

    adjusted_vz = float(command.vz)
    current_altitude = float(current_altitude_m)
    predicted_z = current_altitude + (adjusted_vz * horizon_s)

    if min_z_m is not None and predicted_z < float(min_z_m):
        min_safe_vz = (float(min_z_m) - current_altitude) / horizon_s
        adjusted_vz = max(adjusted_vz, min_safe_vz)
        predicted_z = current_altitude + (adjusted_vz * horizon_s)

    if max_z_m is not None and predicted_z > float(max_z_m):
        max_safe_vz = (float(max_z_m) - current_altitude) / horizon_s
        adjusted_vz = min(adjusted_vz, max_safe_vz)

    if adjusted_vz == float(command.vz):
        return command

    return replace(command, vz=adjusted_vz)


def target_bearing_rad(track: TrackSnapshot) -> float:
    return math.atan2(track.y_b_m, max(track.x_b_m, 1e-3))


def world_error_to_body_frame(
    *,
    dx_world_m: float,
    dy_world_m: float,
    yaw_rad: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(float(yaw_rad))
    sin_yaw = math.sin(float(yaw_rad))
    return (
        (float(dx_world_m) * cos_yaw) + (float(dy_world_m) * sin_yaw),
        (-float(dx_world_m) * sin_yaw) + (float(dy_world_m) * cos_yaw),
    )


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
    min_target_distance_m: float,
    require_target_in_front: bool,
    max_abs_target_y_m: float,
    max_abs_target_z_m: float,
    allow_predicted_tracks: bool = False,
    max_predicted_track_age_s: float = 0.0,
    max_predicted_position_uncertainty_m: float = 0.0,
) -> bool:
    if track.source == TRACK_SOURCE_PREDICTED:
        if not allow_predicted_tracks:
            return False
        if track.last_observed_age_s > max(max_predicted_track_age_s, 0.0):
            return False
        if (
            max_predicted_position_uncertainty_m > 0.0
            and track.position_uncertainty_m > max_predicted_position_uncertainty_m
        ):
            return False
    if track.confidence < min_track_confidence:
        return False
    if track.distance_m < max(float(min_target_distance_m), 0.0):
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


def clamp_follow_command_velocity(
    command: FollowCommand,
    *,
    max_vel_xy_mps: float,
    max_vel_z_mps: float,
    max_yaw_rate_rps: float,
) -> FollowCommand:
    capped_vx = float(command.vx)
    capped_vy = float(command.vy)
    xy_norm = math.hypot(capped_vx, capped_vy)
    xy_cap = max(float(max_vel_xy_mps), 0.0)
    if xy_cap > 0.0 and xy_norm > xy_cap:
        scale = xy_cap / max(xy_norm, 1e-6)
        capped_vx *= scale
        capped_vy *= scale
    z_cap = max(float(max_vel_z_mps), 0.0)
    yaw_cap = max(float(max_yaw_rate_rps), 0.0)
    capped_vz = clamp(command.vz, -z_cap, z_cap)
    capped_yaw_rate = clamp(command.yaw_rate, -yaw_cap, yaw_cap)
    if (
        capped_vx == command.vx
        and capped_vy == command.vy
        and capped_vz == command.vz
        and capped_yaw_rate == command.yaw_rate
    ):
        return command
    return replace(
        command,
        vx=capped_vx,
        vy=capped_vy,
        vz=capped_vz,
        yaw_rate=capped_yaw_rate,
    )


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


def compute_return_to_point_command(
    *,
    current_xy: tuple[float, float],
    current_altitude_m: float,
    current_yaw_rad: float,
    target_xy: tuple[float, float],
    target_altitude_m: float,
    target_yaw_rad: float,
    xy_deadband_m: float,
    z_deadband_m: float,
    yaw_deadband_rad: float,
    kp_xy: float,
    kp_z: float,
    kp_yaw: float,
    max_vel_xy_mps: float,
    max_vel_z_mps: float,
    max_yaw_rate_rps: float,
) -> FollowCommand:
    dx_world_m = float(target_xy[0]) - float(current_xy[0])
    dy_world_m = float(target_xy[1]) - float(current_xy[1])
    err_forward, err_lateral = world_error_to_body_frame(
        dx_world_m=dx_world_m,
        dy_world_m=dy_world_m,
        yaw_rad=current_yaw_rad,
    )
    err_forward = apply_deadband(err_forward, xy_deadband_m)
    err_lateral = apply_deadband(err_lateral, xy_deadband_m)
    err_vertical = apply_deadband(
        float(target_altitude_m) - float(current_altitude_m),
        z_deadband_m,
    )
    yaw_error = apply_deadband(
        wrap_angle_rad(float(target_yaw_rad) - float(current_yaw_rad)),
        yaw_deadband_rad,
    )

    return FollowCommand(
        vx=clamp(kp_xy * err_forward, -max_vel_xy_mps, max_vel_xy_mps),
        vy=clamp(kp_xy * err_lateral, -max_vel_xy_mps, max_vel_xy_mps),
        vz=clamp(kp_z * err_vertical, -max_vel_z_mps, max_vel_z_mps),
        yaw_rate=clamp(
            kp_yaw * yaw_error,
            -max_yaw_rate_rps,
            max_yaw_rate_rps,
        ),
        err_forward=err_forward,
        err_lateral=err_lateral,
        err_vertical=err_vertical,
        yaw_error=yaw_error,
        min_distance_gate_active=False,
    )
