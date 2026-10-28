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


def test_moving_target_prediction_compensates_moving_ownship() -> None:
    memory = TargetMemory(max_prediction_horizon_s=2.0)
    memory.update([make_observation(track_id=2, x_m=5.0, vx_mps=1.0, now_s=0.0)],
                  now_s=0.0)
    predicted = memory.snapshot(now_s=1.0, observation_fresh_s=0.25)[0]
    rotation = np.eye(3)
    relative = world_to_body_position(predicted.position_m,
                                      np.array([1.0, 0.0, 0.0]), rotation)
    assert relative[0] == pytest.approx(5.0)
    assert predicted.output_source(1.0, observation_fresh_s=0.25) == TRACK_SOURCE_PREDICTED


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


def test_zed_local_track_requires_three_coherent_observations():
    memory = TargetMemory(max_observation_speed_mps=2.0,
                          position_noise_allowance_m=0.5,
                          confirmation_count=3, confirmation_span_s=0.2,
                          max_velocity_mps=2.0)
    for t, x in ((0.0, 5.0), (0.1, 5.1)):
        memory.update([make_observation(track_id=4, x_m=x, now_s=t)], now_s=t)
        assert memory.snapshot(now_s=t, observation_fresh_s=0.25) == []
    memory.update([make_observation(track_id=4, x_m=5.2, now_s=0.2)], now_s=0.2)
    tracks = memory.snapshot(now_s=0.2, observation_fresh_s=0.25)
    assert len(tracks) == 1
    assert tracks[0].map_id == 1

    # The detector ID stays the same, but a 5 m to 19 m depth hop is not motion.
    memory.update([make_observation(track_id=4, x_m=19.0, vx_mps=140.0,
                                    now_s=0.3)], now_s=0.3)
    tracks = memory.snapshot(now_s=0.3, observation_fresh_s=0.25)
    assert len(tracks) == 1
    assert tracks[0].map_id == 1
    assert tracks[0].position_m[0] < 6.0
    assert all(np.linalg.norm(track.velocity_mps) <= 2.0 for track in memory.tracks.values())


def test_zed_local_accepts_one_mps_target_with_simultaneous_ownship_motion():
    memory = TargetMemory(max_observation_speed_mps=2.0,
                          position_noise_allowance_m=0.5,
                          confirmation_count=3, confirmation_span_s=0.2,
                          max_velocity_mps=2.0)
    for index in range(6):
        t = index * 0.1
        target_world_x = 5.0 + t
        ownship_world_x = 0.5 * t
        memory.update([make_observation(track_id=8, x_m=target_world_x,
                                        vx_mps=100.0, now_s=t)], now_s=t)
        tracks = memory.snapshot(now_s=t, observation_fresh_s=0.25)
        if index >= 2:
            assert len(tracks) == 1
            relative = world_to_body_position(
                tracks[0].position_m, np.array([ownship_world_x, 0.0, 0.0]),
                np.eye(3))
            assert relative[0] == pytest.approx(5.0 + 0.5 * t, abs=0.2)
            assert np.linalg.norm(tracks[0].velocity_mps) <= 2.0
    assert tracks[0].velocity_mps[0] == pytest.approx(1.0, abs=0.05)


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
