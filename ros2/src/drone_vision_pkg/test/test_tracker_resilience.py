from pathlib import Path
import sys

import numpy as np
import pytest
from norfair import Detection, Tracker

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.tracker_resilience import (
    BodyTrackState,
    compute_lab_histogram_embedding,
    select_held_track_states,
    tracked_object_reid_distance,
)


class StubDetection:
    def __init__(self, embedding):
        self.embedding = embedding


class StubTrack:
    def __init__(self, last_detection, past_detections=None):
        self.last_detection = last_detection
        self.past_detections = [] if past_detections is None else past_detections


def test_histogram_embedding_prefers_similar_appearance() -> None:
    image = np.zeros((40, 40, 3), dtype=np.uint8)
    image[5:25, 5:25] = (0, 0, 255)
    image[5:25, 25:35] = (255, 0, 0)

    same_embedding = compute_lab_histogram_embedding(
        image,
        [5.0, 5.0, 25.0, 25.0],
        bins_per_channel=16,
    )
    shifted_embedding = compute_lab_histogram_embedding(
        image,
        [6.0, 6.0, 24.0, 24.0],
        bins_per_channel=16,
    )
    different_embedding = compute_lab_histogram_embedding(
        image,
        [25.0, 5.0, 35.0, 25.0],
        bins_per_channel=16,
    )

    assert same_embedding is not None
    assert shifted_embedding is not None
    assert different_embedding is not None

    same_track = StubTrack(StubDetection(same_embedding))
    shifted_track = StubTrack(StubDetection(shifted_embedding))
    different_track = StubTrack(StubDetection(different_embedding))

    assert tracked_object_reid_distance(same_track, shifted_track) < 0.05
    assert tracked_object_reid_distance(same_track, different_track) > 0.5


def test_tracked_object_reid_distance_falls_back_to_past_detections() -> None:
    past_embedding = np.array([0.1, 0.9, 0.0], dtype=np.float32)
    matching_embedding = np.array([0.1, 0.9, 0.0], dtype=np.float32)
    nonmatching_embedding = np.array([0.9, 0.1, 0.0], dtype=np.float32)

    first_track = StubTrack(
        StubDetection(None),
        past_detections=[StubDetection(past_embedding)],
    )
    matching_track = StubTrack(StubDetection(matching_embedding))
    nonmatching_track = StubTrack(StubDetection(nonmatching_embedding))

    assert tracked_object_reid_distance(first_track, matching_track) < 0.01
    assert tracked_object_reid_distance(first_track, nonmatching_track) > 1.0


def test_select_held_track_states_caps_extrapolation_and_expires() -> None:
    cached_tracks = {
        7: BodyTrackState(
            track_id=7,
            detector_track_id=7,
            source=0,
            x_b_m=2.0,
            y_b_m=0.1,
            z_b_m=0.0,
            vx_b_mps=2.0,
            vy_b_mps=0.0,
            vz_b_mps=0.0,
            confidence=0.8,
            bbox_area_px=1234.0,
            inbound=True,
            last_seen_s=10.0,
            last_observed_age_s=0.0,
            prediction_horizon_s=0.0,
            position_uncertainty_m=0.1,
            velocity_uncertainty_mps=0.25,
        ),
        9: BodyTrackState(
            track_id=9,
            detector_track_id=9,
            source=0,
            x_b_m=3.0,
            y_b_m=0.0,
            z_b_m=0.0,
            vx_b_mps=0.0,
            vy_b_mps=0.0,
            vz_b_mps=0.0,
            confidence=0.7,
            bbox_area_px=900.0,
            inbound=False,
            last_seen_s=9.0,
            last_observed_age_s=0.0,
            prediction_horizon_s=0.0,
            position_uncertainty_m=0.1,
            velocity_uncertainty_mps=0.25,
        ),
    }

    held_tracks = select_held_track_states(
        active_track_ids={7, 9, 11},
        fresh_track_ids={11},
        cached_tracks=cached_tracks,
        now_s=10.2,
        hold_window_s=0.35,
        max_extrapolation_m=0.25,
    )

    assert set(held_tracks.keys()) == {7}
    assert held_tracks[7].x_b_m == pytest.approx(2.25)
    assert held_tracks[7].y_b_m == pytest.approx(0.1)


def test_norfair_reid_recovers_same_track_id_after_brief_dropout() -> None:
    tracker = Tracker(
        distance_function="euclidean",
        distance_threshold=15,
        initialization_delay=1,
        hit_counter_max=2,
        reid_distance_function=tracked_object_reid_distance,
        reid_distance_threshold=0.2,
        reid_hit_counter_max=5,
    )

    def make_detection(x_px: float, embedding: np.ndarray) -> Detection:
        detection = Detection(
            points=np.array([x_px, 100.0], dtype=np.float32),
            scores=np.array([0.9], dtype=np.float32),
            data={
                "bbox": [x_px - 5.0, 95.0, x_px + 5.0, 105.0],
                "bbox_area_px": 100.0,
                "confidence": 0.9,
            },
        )
        detection.embedding = embedding
        return detection

    red_embedding = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    tracker.update([make_detection(100.0, red_embedding)])
    active_tracks = tracker.update([make_detection(102.0, red_embedding)])
    assert [track.id for track in active_tracks] == [1]

    tracker.update([])
    tracker.update([])
    active_tracks = tracker.update([make_detection(105.0, red_embedding)])

    assert [track.id for track in active_tracks] == [1]
