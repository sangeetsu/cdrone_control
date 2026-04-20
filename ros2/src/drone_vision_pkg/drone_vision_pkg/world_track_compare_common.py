"""Shared helpers for world-track comparison logging and reporting."""

from __future__ import annotations

from dataclasses import dataclass
import math
import re
import time


RUN_MANIFEST_FILENAME = "run_manifest.json"
TRACKS_CSV_FILENAME_DEFAULT = "world_track_compare_tracks.csv"
FRAMES_CSV_FILENAME = "world_track_compare_frames.csv"
REPORT_DIRNAME = "report"

TRACKS_CSV_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "stamp_sec",
    "stamp_nanosec",
    "timestamp_s",
    "track_id",
    "world_frame_id",
    "reference_frame_id",
    "reference_available",
    "ownship_pose_available",
    "frame_match",
    "world_tracks_count",
    "tracker_fps",
    "inference_latency_ms",
    "world_x_m",
    "world_y_m",
    "world_z_m",
    "rigidbody2_x_m",
    "rigidbody2_y_m",
    "rigidbody2_z_m",
    "error_x_m",
    "error_y_m",
    "error_z_m",
    "error_norm_m",
    "body_x_m",
    "body_y_m",
    "body_z_m",
    "truth_body_x_m",
    "truth_body_y_m",
    "truth_body_z_m",
    "body_error_x_m",
    "body_error_y_m",
    "body_error_z_m",
    "truth_range_m",
    "truth_depth_z_m",
    "confidence",
    "distance_m",
    "bbox_area_px",
    "yolo_bbox_x1_px",
    "yolo_bbox_y1_px",
    "yolo_bbox_x2_px",
    "yolo_bbox_y2_px",
    "norfair_bbox_x1_px",
    "norfair_bbox_y1_px",
    "norfair_bbox_x2_px",
    "norfair_bbox_y2_px",
    "yolo_x_px",
    "yolo_y_px",
    "norfair_x_px",
    "norfair_y_px",
    "depth_z_m",
]

FRAMES_CSV_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "stamp_sec",
    "stamp_nanosec",
    "timestamp_s",
    "world_frame_id",
    "reference_frame_id",
    "reference_available",
    "ownship_pose_available",
    "frame_match",
    "truth_range_m",
    "truth_depth_z_m",
    "detections_count",
    "active_tracks_count",
    "world_tracks_count",
    "has_controller_valid_track",
    "best_track_id",
    "best_track_score",
    "best_track_confidence",
    "best_track_distance_m",
    "tracker_fps",
    "inference_latency_ms",
]

FOLLOW_CONTROLLER_GATES = {
    "min_track_confidence": 0.35,
    "max_target_distance_m": 8.0,
    "require_target_in_front": True,
    "max_abs_target_y_m": 4.0,
    "max_abs_target_z_m": 2.5,
    "follow_error_p90_x_m": 0.25,
    "follow_error_p90_y_m": 0.15,
    "follow_error_p90_z_m": 0.15,
    "controller_valid_ratio_min": 0.90,
    "longest_gap_max_s": 0.25,
}


@dataclass(frozen=True)
class ControllerTrackSample:
    """Track fields needed to mirror the follow controller selection logic."""

    track_id: int
    x_b_m: float
    y_b_m: float
    z_b_m: float
    vx_b_mps: float
    vy_b_mps: float
    vz_b_mps: float
    distance_m: float
    confidence: float
    bbox_area_px: float = 0.0
    inbound: bool = False


def sanitize_experiment_tag(tag: str) -> str:
    """Return a filesystem-safe experiment tag."""

    clean_tag = re.sub(r"[^A-Za-z0-9_-]+", "_", str(tag or "").strip())
    return clean_tag.strip("_")


def make_run_id(
    *,
    started_wall_time_s: float | None = None,
    experiment_tag: str = "",
) -> str:
    """Create a stable run identifier for output bundle directories."""

    if started_wall_time_s is None:
        started_wall_time_s = time.time()
    timestamp = time.strftime(
        "%Y%m%d_%H%M%S",
        time.localtime(float(started_wall_time_s)),
    )
    clean_tag = sanitize_experiment_tag(experiment_tag)
    if not clean_tag:
        return timestamp
    return f"{timestamp}_{clean_tag}"


def csv_value(value):
    """Return an empty string for missing CSV values."""

    return "" if value is None else value


def controller_track_is_valid(
    track: ControllerTrackSample,
    *,
    min_track_confidence: float = FOLLOW_CONTROLLER_GATES["min_track_confidence"],
    max_target_distance_m: float = FOLLOW_CONTROLLER_GATES["max_target_distance_m"],
    require_target_in_front: bool = FOLLOW_CONTROLLER_GATES[
        "require_target_in_front"
    ],
    max_abs_target_y_m: float = FOLLOW_CONTROLLER_GATES["max_abs_target_y_m"],
    max_abs_target_z_m: float = FOLLOW_CONTROLLER_GATES["max_abs_target_z_m"],
) -> bool:
    """Apply the controller's track validity gates to a candidate."""

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


def controller_track_score(track: ControllerTrackSample) -> float:
    """Score a candidate track the same way as the follow controller."""

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


def select_best_controller_track(
    tracks: list[ControllerTrackSample],
) -> tuple[ControllerTrackSample | None, float | None]:
    """Return the controller-valid track with the highest selection score."""

    valid_tracks = [track for track in tracks if controller_track_is_valid(track)]
    if not valid_tracks:
        return None, None
    scored_tracks = [
        (controller_track_score(track), track)
        for track in valid_tracks
    ]
    scored_tracks.sort(
        key=lambda item: (
            -item[0],
            item[1].distance_m,
            -item[1].confidence,
        )
    )
    best_score, best_track = scored_tracks[0]
    return best_track, best_score
