from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.follow_utils import TRACK_SOURCE_PREDICTED, TrackSnapshot
from drone_control_pkg.milestone2_demo_logic import (
    project_body_velocity_to_world_xy,
    select_sequential_target,
    update_dwell_progress,
)


def make_track(
    track_id: int,
    *,
    x_b_m: float,
    y_b_m: float = 0.0,
    z_b_m: float = 0.0,
    vx_b_mps: float = 0.0,
    vy_b_mps: float = 0.0,
    vz_b_mps: float = 0.0,
    distance_m: float | None = None,
    confidence: float = 0.9,
    last_seen_s: float = 10.0,
) -> TrackSnapshot:
    return TrackSnapshot(
        track_id=track_id,
        x_b_m=x_b_m,
        y_b_m=y_b_m,
        z_b_m=z_b_m,
        vx_b_mps=vx_b_mps,
        vy_b_mps=vy_b_mps,
        vz_b_mps=vz_b_mps,
        distance_m=x_b_m if distance_m is None else distance_m,
        confidence=confidence,
        bbox_area_px=1000.0,
        inbound=vx_b_mps < 0.0,
        last_seen_s=last_seen_s,
    )


def test_select_sequential_target_keeps_active_lock_when_valid():
    tracks = [
        make_track(11, x_b_m=1.8, confidence=0.8),
        make_track(22, x_b_m=1.5, confidence=0.95),
    ]

    chosen = select_sequential_target(
        tracks,
        active_track_id=11,
        excluded_track_ids=set(),
        now_s=10.2,
        track_timeout_s=0.5,
        min_track_confidence=0.35,
        max_target_distance_m=2.6,
        min_target_distance_m=0.0,
        require_target_in_front=True,
        max_abs_target_y_m=4.0,
        max_abs_target_z_m=2.5,
    )

    assert chosen is not None
    assert chosen.track_id == 11


def test_select_sequential_target_skips_excluded_and_stale_tracks():
    tracks = [
        make_track(11, x_b_m=1.6, confidence=0.9, last_seen_s=10.0),
        make_track(22, x_b_m=1.4, confidence=0.9, last_seen_s=9.0),
        make_track(33, x_b_m=1.7, confidence=0.7, last_seen_s=10.0),
    ]

    chosen = select_sequential_target(
        tracks,
        active_track_id=None,
        excluded_track_ids={11},
        now_s=10.3,
        track_timeout_s=0.5,
        min_track_confidence=0.35,
        max_target_distance_m=2.6,
        min_target_distance_m=0.0,
        require_target_in_front=True,
        max_abs_target_y_m=4.0,
        max_abs_target_z_m=2.5,
    )

    assert chosen is not None
    assert chosen.track_id == 33


def test_select_sequential_target_accepts_milestone4_predicted_track_options():
    predicted = make_track(44, x_b_m=1.9, confidence=0.9)
    predicted.source = TRACK_SOURCE_PREDICTED
    predicted.last_observed_age_s = 0.2
    predicted.position_uncertainty_m = 0.3

    chosen = select_sequential_target(
        [predicted],
        active_track_id=None,
        excluded_track_ids=set(),
        now_s=10.2,
        track_timeout_s=0.5,
        min_track_confidence=0.35,
        max_target_distance_m=2.6,
        min_target_distance_m=0.0,
        require_target_in_front=True,
        max_abs_target_y_m=4.0,
        max_abs_target_z_m=2.5,
        allow_predicted_tracks=True,
        max_predicted_track_age_s=0.5,
        max_predicted_position_uncertainty_m=0.5,
    )

    assert chosen is predicted


def test_select_sequential_target_rejects_implausibly_close_tracks():
    close_track = make_track(55, x_b_m=0.02, confidence=0.9, distance_m=0.02)
    valid_track = make_track(66, x_b_m=1.2, confidence=0.7, distance_m=1.2)

    chosen = select_sequential_target(
        [close_track, valid_track],
        active_track_id=None,
        excluded_track_ids=set(),
        now_s=10.2,
        track_timeout_s=0.5,
        min_track_confidence=0.35,
        max_target_distance_m=2.6,
        min_target_distance_m=0.5,
        require_target_in_front=True,
        max_abs_target_y_m=4.0,
        max_abs_target_z_m=2.5,
    )

    assert chosen is valid_track


def test_update_dwell_progress_completes_after_required_duration():
    dwell_started_s, elapsed_s, remaining_s, complete = update_dwell_progress(
        None,
        in_standoff_window=True,
        now_s=5.0,
        dwell_time_s=3.0,
    )

    assert dwell_started_s == 5.0
    assert elapsed_s == 0.0
    assert remaining_s == 3.0
    assert complete is False

    dwell_started_s, elapsed_s, remaining_s, complete = update_dwell_progress(
        dwell_started_s,
        in_standoff_window=True,
        now_s=8.1,
        dwell_time_s=3.0,
    )

    assert dwell_started_s == 5.0
    assert elapsed_s == pytest.approx(3.1)
    assert remaining_s == pytest.approx(0.0)
    assert complete is True


def test_project_body_velocity_to_world_xy_rotates_body_axes():
    projected_xy = project_body_velocity_to_world_xy(
        current_xy=(1.0, 2.0),
        yaw_rad=1.5707963267948966,
        vx_mps=0.5,
        vy_mps=0.0,
        horizon_s=2.0,
    )

    assert projected_xy[0] == pytest.approx(1.0)
    assert projected_xy[1] == pytest.approx(3.0)
