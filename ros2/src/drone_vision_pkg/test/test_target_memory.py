from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.target_memory import (
    TRACK_SOURCE_DETECTED,
    TRACK_SOURCE_PREDICTED,
    TargetMemory,
    WorldTrackObservation,
    normalized_quaternion_to_rotation_matrix,
    world_to_body_position,
)


def make_observation(
    *,
    track_id: int,
    detector_track_id: int | None = None,
    x_m: float,
    y_m: float = 0.0,
    z_m: float = 1.0,
    vx_mps: float = 0.0,
    vy_mps: float = 0.0,
    vz_mps: float = 0.0,
    confidence: float = 0.9,
    now_s: float = 0.0,
) -> WorldTrackObservation:
    return WorldTrackObservation(
        track_id=track_id,
        detector_track_id=track_id if detector_track_id is None else detector_track_id,
        source=TRACK_SOURCE_DETECTED,
        position_m=np.array([x_m, y_m, z_m], dtype=np.float64),
        velocity_mps=np.array([vx_mps, vy_mps, vz_mps], dtype=np.float64),
        confidence=confidence,
        bbox_area_px=1200.0,
        distance_m=float(np.linalg.norm([x_m, y_m, z_m])),
        inbound=False,
        observed_at_s=now_s,
        received_at_s=now_s,
        last_observed_age_s=0.0,
        prediction_horizon_s=0.0,
        position_uncertainty_m=0.12,
        velocity_uncertainty_mps=0.25,
    )


def test_target_memory_predicts_during_dropout_with_decay_and_uncertainty() -> None:
    memory = TargetMemory(
        confidence_decay_per_s=0.25,
        position_process_noise_mps=0.4,
        max_prediction_horizon_s=3.0,
    )
    memory.update(
        [make_observation(track_id=7, x_m=0.0, vx_mps=1.0, now_s=0.0)],
        now_s=0.0,
    )

    tracks = memory.snapshot(now_s=1.0, observation_fresh_s=0.25)

    assert len(tracks) == 1
    track = tracks[0]
    assert track.map_id == 1
    assert track.output_source(1.0, observation_fresh_s=0.25) == TRACK_SOURCE_PREDICTED
    assert track.position_m[0] == pytest.approx(1.0)
    assert track.decayed_confidence(1.0, confidence_decay_per_s=0.25) == pytest.approx(
        0.675
    )
    assert track.position_uncertainty_m > 0.12


def test_target_memory_reacquires_near_prediction_with_stable_map_id() -> None:
    memory = TargetMemory(association_gate_m=0.8, max_prediction_horizon_s=3.0)
    memory.update(
        [make_observation(track_id=7, x_m=0.0, vx_mps=1.0, now_s=0.0)],
        now_s=0.0,
    )
    memory.snapshot(now_s=1.0, observation_fresh_s=0.25)

    memory.update(
        [
            make_observation(
                track_id=99,
                detector_track_id=99,
                x_m=1.1,
                vx_mps=1.0,
                now_s=1.1,
            )
        ],
        now_s=1.1,
    )

    tracks = memory.snapshot(now_s=1.1, observation_fresh_s=0.25)
    assert len(tracks) == 1
    assert tracks[0].map_id == 1
    assert tracks[0].detector_track_id == 99
    assert tracks[0].observed_count == 2


def test_target_memory_prunes_stale_tracks_after_prediction_window() -> None:
    memory = TargetMemory(max_prediction_horizon_s=1.0, prune_after_s=2.0)
    memory.update(
        [make_observation(track_id=3, x_m=0.0, now_s=0.0)],
        now_s=0.0,
    )

    assert memory.snapshot(now_s=1.1, observation_fresh_s=0.25) == []
    memory.prune(2.1)
    assert memory.tracks == {}


def test_world_to_body_position_uses_inverse_ownship_rotation() -> None:
    rotation = normalized_quaternion_to_rotation_matrix(
        0.0,
        0.0,
        0.70710678118,
        0.70710678118,
    )

    body_position = world_to_body_position(
        np.array([1.0, 0.0, 2.0], dtype=np.float64),
        np.array([0.0, 0.0, 1.0], dtype=np.float64),
        rotation,
    )

    assert body_position[0] == pytest.approx(0.0, abs=1e-6)
    assert body_position[1] == pytest.approx(-1.0, abs=1e-6)
    assert body_position[2] == pytest.approx(1.0, abs=1e-6)
