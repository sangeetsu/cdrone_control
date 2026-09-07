from __future__ import annotations

import math
from typing import Iterable, Optional

from drone_control_pkg.follow_utils import TrackSnapshot, score_track, track_is_valid


def select_sequential_target(
    tracks: Iterable[TrackSnapshot],
    *,
    active_track_id: Optional[int],
    excluded_track_ids: set[int],
    now_s: float,
    track_timeout_s: float,
    min_track_confidence: float,
    max_target_distance_m: float,
    min_target_distance_m: float,
    require_target_in_front: bool,
    max_abs_target_y_m: float,
    max_abs_target_z_m: float,
    allow_predicted_tracks: bool = False,
    max_predicted_track_age_s: float = 0.0,
    max_predicted_position_uncertainty_m: float = 0.0,
) -> Optional[TrackSnapshot]:
    def is_candidate(track: TrackSnapshot) -> bool:
        if track.track_id in excluded_track_ids:
            return False
        if now_s - track.last_seen_s > track_timeout_s:
            return False
        return track_is_valid(
            track,
            min_track_confidence=min_track_confidence,
            min_target_distance_m=min_target_distance_m,
            max_target_distance_m=max_target_distance_m,
            require_target_in_front=require_target_in_front,
            max_abs_target_y_m=max_abs_target_y_m,
            max_abs_target_z_m=max_abs_target_z_m,
            allow_predicted_tracks=allow_predicted_tracks,
            max_predicted_track_age_s=max_predicted_track_age_s,
            max_predicted_position_uncertainty_m=(
                max_predicted_position_uncertainty_m
            ),
        )

    current = None
    if active_track_id is not None:
        for track in tracks:
            if track.track_id == active_track_id:
                current = track
                break
    if current is not None and is_candidate(current):
        return current

    candidates = [track for track in tracks if is_candidate(track)]
    if not candidates:
        return None

    candidates.sort(
        key=lambda track: (
            -score_track(track),
            track.distance_m,
            -track.confidence,
        )
    )
    return candidates[0]


def update_dwell_progress(
    dwell_started_s: Optional[float],
    *,
    in_standoff_window: bool,
    now_s: float,
    dwell_time_s: float,
) -> tuple[Optional[float], float, float, bool]:
    if not in_standoff_window:
        return None, 0.0, max(float(dwell_time_s), 0.0), False

    if dwell_started_s is None:
        dwell_started_s = float(now_s)

    elapsed_s = max(float(now_s) - float(dwell_started_s), 0.0)
    required_s = max(float(dwell_time_s), 0.0)
    complete = elapsed_s >= required_s
    remaining_s = max(required_s - elapsed_s, 0.0)
    return dwell_started_s, elapsed_s, remaining_s, complete


def project_body_velocity_to_world_xy(
    *,
    current_xy: tuple[float, float],
    yaw_rad: float,
    vx_mps: float,
    vy_mps: float,
    horizon_s: float,
) -> tuple[float, float]:
    dt_s = max(float(horizon_s), 0.0)
    cos_yaw = math.cos(float(yaw_rad))
    sin_yaw = math.sin(float(yaw_rad))
    delta_x = (float(vx_mps) * cos_yaw) - (float(vy_mps) * sin_yaw)
    delta_y = (float(vx_mps) * sin_yaw) + (float(vy_mps) * cos_yaw)
    return (
        float(current_xy[0]) + (delta_x * dt_s),
        float(current_xy[1]) + (delta_y * dt_s),
    )
