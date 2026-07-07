from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import re
import time
from typing import Any, Iterable

import numpy as np
import rclpy
from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_ownship_pose_topic,
)
from drone_control_pkg.topic_utils import cdrone_topic
from drone_msgs.msg import EngagementState, WorldTargetTrack, WorldTargetTrackArray
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    import cv2
except ImportError:  # pragma: no cover - exercised only on systems without OpenCV
    cv2 = None


TRACK_SOURCE_DETECTED = 0
TRACK_SOURCE_HELD = 1
TRACK_SOURCE_PREDICTED = 2
MISSION_TERMINAL_STATES = {"IDLE", "COMPLETE", "ABORT"}

SAMPLES_CSV = "tracking_samples.csv"
FRAMES_CSV = "tracking_frames.csv"
CLEANED_PATH_CSV = "cleaned_target_path.csv"
SUMMARY_CSV = "tracking_summary.csv"
MANIFEST_JSON = "tracking_metrics_manifest.json"
CLEANED_PATH_MP4 = "cleaned_target_path.mp4"
REFERENCE_VIDEO_PATTERN = "mission_*_combined.mp4"
MAX_REFERENCE_START_DELTA_S = 120.0
CLEANED_VIDEO_HISTORY_WINDOW_S = 10.0

SAMPLE_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "timestamp_s",
    "sample_kind",
    "track_id",
    "detector_track_id",
    "source",
    "source_label",
    "x_m",
    "y_m",
    "z_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "confidence",
    "distance_m",
    "bbox_area_px",
    "inbound",
    "last_observed_age_s",
    "prediction_horizon_s",
    "position_uncertainty_m",
    "velocity_uncertainty_mps",
]

FRAME_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "timestamp_s",
    "state",
    "raw_tracks_count",
    "target_map_tracks_count",
    "raw_has_track",
    "target_map_has_track",
    "target_map_predicted_count",
    "target_map_has_predicted",
    "active_raw_track_id",
    "active_target_map_track_id",
    "raw_dropout",
    "target_map_bridge_active",
    "raw_dropout_duration_s",
    "target_map_bridge_duration_s",
    "longest_raw_dropout_s",
    "longest_target_map_bridge_s",
]

CLEANED_PATH_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "timestamp_s",
    "sample_kind",
    "track_id",
    "source",
    "source_label",
    "x_m",
    "y_m",
    "z_m",
    "cleaned_x_m",
    "cleaned_y_m",
    "cleaned_z_m",
    "confidence",
    "position_uncertainty_m",
]

SUMMARY_FIELDNAMES = [
    "run_id",
    "experiment_tag",
    "duration_s",
    "frame_count",
    "raw_track_frame_count",
    "target_map_track_frame_count",
    "target_map_predicted_frame_count",
    "raw_visual_track_coverage_ratio",
    "target_map_coverage_ratio",
    "predicted_coverage_ratio",
    "predicted_coverage_during_raw_dropout_ratio",
    "longest_raw_dropout_s",
    "longest_no_track_gap_s",
    "longest_target_map_bridge_s",
    "raw_id_switch_count",
    "target_map_id_switch_count",
    "raw_reacquisition_count",
    "target_map_same_id_reacquisition_count",
    "target_map_same_id_reacquisition_ratio",
    "reacquisition_residual_count",
    "reacquisition_residual_median_m",
    "reacquisition_residual_p90_m",
    "uncertainty_calibration_ratio",
    "simulated_dropout_count",
    "simulated_dropout_error_median_m",
    "simulated_dropout_error_p90_m",
    "cleaned_path_point_count",
    "cleaned_path_source_kind",
    "cleaned_path_track_id",
]


def is_active_mission_state(state: str) -> bool:
    state_upper = str(state or "").strip().upper()
    return bool(state_upper) and state_upper not in MISSION_TERMINAL_STATES


def source_label(source: int) -> str:
    if int(source) == TRACK_SOURCE_DETECTED:
        return "detected"
    if int(source) == TRACK_SOURCE_HELD:
        return "held"
    if int(source) == TRACK_SOURCE_PREDICTED:
        return "predicted"
    return "unknown"


def finite_float(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def optional_float(value: Any) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    return number if math.isfinite(number) else None


def safe_ratio(numerator: int | float, denominator: int | float) -> float | None:
    denominator = float(denominator)
    if denominator <= 0.0:
        return None
    return float(numerator) / denominator


def percentile(values: Iterable[float], q: float) -> float | None:
    finite_values = [
        float(value)
        for value in values
        if math.isfinite(float(value))
    ]
    if not finite_values:
        return None
    return float(np.percentile(np.asarray(finite_values, dtype=np.float64), q))


def stamp_to_s(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0)) + (
        float(getattr(stamp, "nanosec", 0)) * 1e-9
    )


def make_run_id(
    *,
    drone_id: str,
    experiment_tag: str,
    started_wall_time_s: float | None = None,
) -> str:
    if started_wall_time_s is None:
        started_wall_time_s = time.time()
    timestamp = time.strftime(
        "%Y%m%d_%H%M%S",
        time.localtime(float(started_wall_time_s)),
    )
    millis = int((float(started_wall_time_s) % 1.0) * 1000.0)
    safe_drone = "".join(
        ch.lower() if ch.isalnum() else "_" for ch in str(drone_id or "drone")
    ).strip("_")
    safe_tag = "".join(
        ch.lower() if ch.isalnum() else "_" for ch in str(experiment_tag or "")
    ).strip("_")
    parts = ["tracking", safe_drone or "drone", f"{timestamp}_{millis:03d}"]
    if safe_tag:
        parts.append(safe_tag)
    return "_".join(parts)


@dataclass(frozen=True)
class TrackSample:
    timestamp_s: float
    sample_kind: str
    track_id: int
    detector_track_id: int
    source: int
    position_m: np.ndarray
    velocity_mps: np.ndarray
    confidence: float
    distance_m: float
    bbox_area_px: float
    inbound: bool
    last_observed_age_s: float
    prediction_horizon_s: float
    position_uncertainty_m: float
    velocity_uncertainty_mps: float


@dataclass(frozen=True)
class FrameSample:
    timestamp_s: float
    state: str
    raw_tracks_count: int
    target_map_tracks_count: int
    raw_has_track: bool
    target_map_has_track: bool
    target_map_predicted_count: int
    target_map_has_predicted: bool
    active_raw_track_id: int | None
    active_target_map_track_id: int | None
    raw_dropout: bool
    target_map_bridge_active: bool
    raw_dropout_duration_s: float
    target_map_bridge_duration_s: float
    longest_raw_dropout_s: float
    longest_target_map_bridge_s: float


@dataclass(frozen=True)
class CleanedPathPoint:
    timestamp_s: float
    sample_kind: str
    track_id: int
    source: int
    position_m: np.ndarray
    cleaned_position_m: np.ndarray
    confidence: float
    position_uncertainty_m: float


@dataclass(frozen=True)
class ReferenceVideoInfo:
    path: Path
    fps: float
    frame_count: int
    width: int
    height: int

    @property
    def duration_s(self) -> float:
        if self.fps <= 0.0:
            return 0.0
        return float(self.frame_count) / float(self.fps)


@dataclass(frozen=True)
class CleanedTrackPostprocessResult:
    cleaned_path_csv: Path
    cleaned_path_mp4: Path | None
    time_origin_s: float
    time_origin_source: str
    reference_video: ReferenceVideoInfo | None
    video_synced_to_reference: bool
    video_fps: float
    video_frame_count: int
    fallback_reason: str

    def to_manifest_dict(self) -> dict[str, object]:
        reference = self.reference_video
        return {
            "cleaned_path_csv": str(self.cleaned_path_csv),
            "cleaned_path_mp4": "" if self.cleaned_path_mp4 is None else str(self.cleaned_path_mp4),
            "time_origin_s": self.time_origin_s,
            "time_origin_source": self.time_origin_source,
            "reference_video_path": "" if reference is None else str(reference.path),
            "reference_video_fps": "" if reference is None else reference.fps,
            "reference_video_frame_count": 0 if reference is None else reference.frame_count,
            "reference_video_duration_s": "" if reference is None else reference.duration_s,
            "video_synced_to_reference": self.video_synced_to_reference,
            "video_fps": self.video_fps,
            "video_frame_count": self.video_frame_count,
            "fallback_reason": self.fallback_reason,
        }


def sample_from_world_track(
    track: WorldTargetTrack,
    *,
    timestamp_s: float,
    sample_kind: str,
) -> TrackSample:
    detector_track_id = int(getattr(track, "detector_track_id", int(track.track_id)))
    return TrackSample(
        timestamp_s=float(timestamp_s),
        sample_kind=str(sample_kind),
        track_id=int(track.track_id),
        detector_track_id=detector_track_id,
        source=int(getattr(track, "source", TRACK_SOURCE_DETECTED)),
        position_m=np.array(
            [
                finite_float(getattr(track, "x_m", 0.0)),
                finite_float(getattr(track, "y_m", 0.0)),
                finite_float(getattr(track, "z_m", 0.0)),
            ],
            dtype=np.float64,
        ),
        velocity_mps=np.array(
            [
                finite_float(getattr(track, "vx_mps", 0.0)),
                finite_float(getattr(track, "vy_mps", 0.0)),
                finite_float(getattr(track, "vz_mps", 0.0)),
            ],
            dtype=np.float64,
        ),
        confidence=finite_float(getattr(track, "confidence", 0.0)),
        distance_m=finite_float(getattr(track, "distance_m", 0.0)),
        bbox_area_px=finite_float(getattr(track, "bbox_area_px", 0.0)),
        inbound=bool(getattr(track, "inbound", False)),
        last_observed_age_s=max(
            finite_float(getattr(track, "last_observed_age_s", 0.0)),
            0.0,
        ),
        prediction_horizon_s=max(
            finite_float(getattr(track, "prediction_horizon_s", 0.0)),
            0.0,
        ),
        position_uncertainty_m=max(
            finite_float(getattr(track, "position_uncertainty_m", 0.0)),
            0.0,
        ),
        velocity_uncertainty_mps=max(
            finite_float(getattr(track, "velocity_uncertainty_mps", 0.0)),
            0.0,
        ),
    )


def select_active_track(samples: list[TrackSample]) -> TrackSample | None:
    if not samples:
        return None
    return sorted(
        samples,
        key=lambda sample: (
            -float(sample.confidence),
            float(sample.distance_m) if sample.distance_m > 0.0 else float("inf"),
            int(sample.track_id),
        ),
    )[0]


def sample_to_csv_row(
    sample: TrackSample,
    *,
    run_id: str,
    experiment_tag: str,
) -> dict[str, object]:
    return {
        "run_id": run_id,
        "experiment_tag": experiment_tag,
        "timestamp_s": f"{sample.timestamp_s:.9f}",
        "sample_kind": sample.sample_kind,
        "track_id": sample.track_id,
        "detector_track_id": sample.detector_track_id,
        "source": sample.source,
        "source_label": source_label(sample.source),
        "x_m": f"{sample.position_m[0]:.6f}",
        "y_m": f"{sample.position_m[1]:.6f}",
        "z_m": f"{sample.position_m[2]:.6f}",
        "vx_mps": f"{sample.velocity_mps[0]:.6f}",
        "vy_mps": f"{sample.velocity_mps[1]:.6f}",
        "vz_mps": f"{sample.velocity_mps[2]:.6f}",
        "confidence": f"{sample.confidence:.6f}",
        "distance_m": f"{sample.distance_m:.6f}",
        "bbox_area_px": f"{sample.bbox_area_px:.3f}",
        "inbound": int(sample.inbound),
        "last_observed_age_s": f"{sample.last_observed_age_s:.6f}",
        "prediction_horizon_s": f"{sample.prediction_horizon_s:.6f}",
        "position_uncertainty_m": f"{sample.position_uncertainty_m:.6f}",
        "velocity_uncertainty_mps": f"{sample.velocity_uncertainty_mps:.6f}",
    }


def frame_to_csv_row(
    frame: FrameSample,
    *,
    run_id: str,
    experiment_tag: str,
) -> dict[str, object]:
    return {
        "run_id": run_id,
        "experiment_tag": experiment_tag,
        "timestamp_s": f"{frame.timestamp_s:.9f}",
        "state": frame.state,
        "raw_tracks_count": frame.raw_tracks_count,
        "target_map_tracks_count": frame.target_map_tracks_count,
        "raw_has_track": int(frame.raw_has_track),
        "target_map_has_track": int(frame.target_map_has_track),
        "target_map_predicted_count": frame.target_map_predicted_count,
        "target_map_has_predicted": int(frame.target_map_has_predicted),
        "active_raw_track_id": "" if frame.active_raw_track_id is None else frame.active_raw_track_id,
        "active_target_map_track_id": (
            ""
            if frame.active_target_map_track_id is None
            else frame.active_target_map_track_id
        ),
        "raw_dropout": int(frame.raw_dropout),
        "target_map_bridge_active": int(frame.target_map_bridge_active),
        "raw_dropout_duration_s": f"{frame.raw_dropout_duration_s:.6f}",
        "target_map_bridge_duration_s": f"{frame.target_map_bridge_duration_s:.6f}",
        "longest_raw_dropout_s": f"{frame.longest_raw_dropout_s:.6f}",
        "longest_target_map_bridge_s": f"{frame.longest_target_map_bridge_s:.6f}",
    }


class TrackingMetricsRun:
    def __init__(
        self,
        *,
        run_id: str,
        experiment_tag: str,
        max_reacquisition_gap_s: float = 3.0,
        simulated_dropout_horizon_s: float = 1.0,
        smoothing_window: int = 5,
    ) -> None:
        self.run_id = str(run_id)
        self.experiment_tag = str(experiment_tag)
        self.max_reacquisition_gap_s = max(float(max_reacquisition_gap_s), 0.0)
        self.simulated_dropout_horizon_s = max(float(simulated_dropout_horizon_s), 0.0)
        self.smoothing_window = max(int(smoothing_window), 1)
        self.samples: list[TrackSample] = []
        self.frames: list[FrameSample] = []
        self.ownship_history: list[tuple[float, np.ndarray]] = []
        self.raw_dropout_start_s: float | None = None
        self.bridge_start_s: float | None = None
        self.longest_raw_dropout_s = 0.0
        self.longest_bridge_s = 0.0
        self.last_raw_track_id: int | None = None
        self.last_target_map_track_id: int | None = None
        self.raw_id_switch_count = 0
        self.target_map_id_switch_count = 0
        self.last_map_id_before_dropout: int | None = None
        self.last_predicted_map_sample: TrackSample | None = None
        self.raw_reacquisition_count = 0
        self.target_map_same_id_reacquisition_count = 0
        self.reacquisition_residuals_m: list[float] = []
        self.reacquisition_uncertainties_m: list[float] = []

    def add_samples(self, samples: list[TrackSample]) -> None:
        self.samples.extend(samples)

    def add_ownship_position(self, timestamp_s: float, position_m: np.ndarray) -> None:
        position = np.asarray(position_m, dtype=np.float64)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            return
        self.ownship_history.append((float(timestamp_s), position.copy()))

    def sample_frame(
        self,
        *,
        timestamp_s: float,
        state: str,
        raw_tracks: list[TrackSample],
        target_map_tracks: list[TrackSample],
    ) -> FrameSample:
        active_raw = select_active_track(raw_tracks)
        active_map = select_active_track(target_map_tracks)
        predicted_map_tracks = [
            sample
            for sample in target_map_tracks
            if int(sample.source) == TRACK_SOURCE_PREDICTED
        ]
        active_predicted_map = select_active_track(predicted_map_tracks)
        raw_has_track = active_raw is not None
        map_has_track = active_map is not None
        map_has_predicted = bool(predicted_map_tracks)
        raw_dropout = not raw_has_track
        bridge_active = raw_dropout and map_has_predicted

        if raw_has_track:
            self._handle_raw_reacquisition(
                timestamp_s=timestamp_s,
                active_raw=active_raw,
                active_map=active_map,
            )
            self.raw_dropout_start_s = None
        elif self.raw_dropout_start_s is None and self.last_raw_track_id is not None:
            self.raw_dropout_start_s = float(timestamp_s)
            self.last_map_id_before_dropout = (
                None if active_map is None else int(active_map.track_id)
            )
            self.last_predicted_map_sample = None

        raw_dropout_duration_s = 0.0
        if self.raw_dropout_start_s is not None:
            raw_dropout_duration_s = max(timestamp_s - self.raw_dropout_start_s, 0.0)
            self.longest_raw_dropout_s = max(
                self.longest_raw_dropout_s,
                raw_dropout_duration_s,
            )

        if bridge_active:
            if self.bridge_start_s is None:
                self.bridge_start_s = float(timestamp_s)
            self.last_predicted_map_sample = active_predicted_map
        else:
            self.bridge_start_s = None

        bridge_duration_s = 0.0
        if self.bridge_start_s is not None:
            bridge_duration_s = max(timestamp_s - self.bridge_start_s, 0.0)
            self.longest_bridge_s = max(self.longest_bridge_s, bridge_duration_s)

        if active_raw is not None:
            if (
                self.last_raw_track_id is not None
                and int(active_raw.track_id) != self.last_raw_track_id
            ):
                self.raw_id_switch_count += 1
            self.last_raw_track_id = int(active_raw.track_id)
        if active_map is not None:
            if (
                self.last_target_map_track_id is not None
                and int(active_map.track_id) != self.last_target_map_track_id
            ):
                self.target_map_id_switch_count += 1
            self.last_target_map_track_id = int(active_map.track_id)

        frame = FrameSample(
            timestamp_s=float(timestamp_s),
            state=str(state),
            raw_tracks_count=len(raw_tracks),
            target_map_tracks_count=len(target_map_tracks),
            raw_has_track=raw_has_track,
            target_map_has_track=map_has_track,
            target_map_predicted_count=len(predicted_map_tracks),
            target_map_has_predicted=map_has_predicted,
            active_raw_track_id=None if active_raw is None else int(active_raw.track_id),
            active_target_map_track_id=(
                None if active_map is None else int(active_map.track_id)
            ),
            raw_dropout=raw_dropout,
            target_map_bridge_active=bridge_active,
            raw_dropout_duration_s=raw_dropout_duration_s,
            target_map_bridge_duration_s=bridge_duration_s,
            longest_raw_dropout_s=self.longest_raw_dropout_s,
            longest_target_map_bridge_s=self.longest_bridge_s,
        )
        self.frames.append(frame)
        return frame

    def _handle_raw_reacquisition(
        self,
        *,
        timestamp_s: float,
        active_raw: TrackSample,
        active_map: TrackSample | None,
    ) -> None:
        if self.raw_dropout_start_s is None:
            return
        dropout_duration_s = max(float(timestamp_s) - self.raw_dropout_start_s, 0.0)
        if dropout_duration_s > self.max_reacquisition_gap_s:
            return

        self.raw_reacquisition_count += 1
        if (
            self.last_map_id_before_dropout is not None
            and active_map is not None
            and int(active_map.track_id) == self.last_map_id_before_dropout
        ):
            self.target_map_same_id_reacquisition_count += 1

        predicted_sample = self.last_predicted_map_sample
        if predicted_sample is None:
            return
        residual_m = float(
            np.linalg.norm(active_raw.position_m - predicted_sample.position_m)
        )
        if math.isfinite(residual_m):
            self.reacquisition_residuals_m.append(residual_m)
            self.reacquisition_uncertainties_m.append(
                max(float(predicted_sample.position_uncertainty_m), 0.0)
            )

    def cleaned_path(self) -> list[CleanedPathPoint]:
        return clean_target_path(
            self.samples,
            smoothing_window=self.smoothing_window,
        )

    def summary(self) -> dict[str, object]:
        frames = self.frames
        frame_count = len(frames)
        raw_track_frame_count = sum(1 for frame in frames if frame.raw_has_track)
        map_track_frame_count = sum(1 for frame in frames if frame.target_map_has_track)
        predicted_frame_count = sum(
            1 for frame in frames if frame.target_map_has_predicted
        )
        raw_dropout_frames = [frame for frame in frames if frame.raw_dropout]
        predicted_during_dropout = sum(
            1
            for frame in raw_dropout_frames
            if frame.target_map_has_predicted
        )
        no_track_gap_s = longest_false_coverage_gap_s(frames)
        calibrated_count = sum(
            1
            for residual, uncertainty in zip(
                self.reacquisition_residuals_m,
                self.reacquisition_uncertainties_m,
            )
            if residual <= uncertainty
        )
        simulated_errors = simulated_dropout_backtest(
            self.samples,
            horizon_s=self.simulated_dropout_horizon_s,
        )
        cleaned_path = self.cleaned_path()
        cleaned_kind = cleaned_path[0].sample_kind if cleaned_path else ""
        cleaned_track_id = cleaned_path[0].track_id if cleaned_path else ""

        duration_s = 0.0
        if frame_count >= 2:
            duration_s = frames[-1].timestamp_s - frames[0].timestamp_s

        return {
            "run_id": self.run_id,
            "experiment_tag": self.experiment_tag,
            "duration_s": f"{max(duration_s, 0.0):.6f}",
            "frame_count": frame_count,
            "raw_track_frame_count": raw_track_frame_count,
            "target_map_track_frame_count": map_track_frame_count,
            "target_map_predicted_frame_count": predicted_frame_count,
            "raw_visual_track_coverage_ratio": _csv_optional(
                safe_ratio(raw_track_frame_count, frame_count)
            ),
            "target_map_coverage_ratio": _csv_optional(
                safe_ratio(map_track_frame_count, frame_count)
            ),
            "predicted_coverage_ratio": _csv_optional(
                safe_ratio(predicted_frame_count, frame_count)
            ),
            "predicted_coverage_during_raw_dropout_ratio": _csv_optional(
                safe_ratio(predicted_during_dropout, len(raw_dropout_frames))
            ),
            "longest_raw_dropout_s": f"{self.longest_raw_dropout_s:.6f}",
            "longest_no_track_gap_s": f"{no_track_gap_s:.6f}",
            "longest_target_map_bridge_s": f"{self.longest_bridge_s:.6f}",
            "raw_id_switch_count": self.raw_id_switch_count,
            "target_map_id_switch_count": self.target_map_id_switch_count,
            "raw_reacquisition_count": self.raw_reacquisition_count,
            "target_map_same_id_reacquisition_count": (
                self.target_map_same_id_reacquisition_count
            ),
            "target_map_same_id_reacquisition_ratio": _csv_optional(
                safe_ratio(
                    self.target_map_same_id_reacquisition_count,
                    self.raw_reacquisition_count,
                )
            ),
            "reacquisition_residual_count": len(self.reacquisition_residuals_m),
            "reacquisition_residual_median_m": _csv_optional(
                percentile(self.reacquisition_residuals_m, 50)
            ),
            "reacquisition_residual_p90_m": _csv_optional(
                percentile(self.reacquisition_residuals_m, 90)
            ),
            "uncertainty_calibration_ratio": _csv_optional(
                safe_ratio(calibrated_count, len(self.reacquisition_residuals_m))
            ),
            "simulated_dropout_count": len(simulated_errors),
            "simulated_dropout_error_median_m": _csv_optional(
                percentile(simulated_errors, 50)
            ),
            "simulated_dropout_error_p90_m": _csv_optional(
                percentile(simulated_errors, 90)
            ),
            "cleaned_path_point_count": len(cleaned_path),
            "cleaned_path_source_kind": cleaned_kind,
            "cleaned_path_track_id": cleaned_track_id,
        }


def _csv_optional(value: float | None) -> str:
    return "" if value is None else f"{float(value):.6f}"


def longest_false_coverage_gap_s(frames: list[FrameSample]) -> float:
    if len(frames) < 2:
        return 0.0
    longest_s = 0.0
    gap_start_s: float | None = None
    last_timestamp_s = frames[0].timestamp_s
    for frame in frames:
        has_any_track = frame.raw_has_track or frame.target_map_has_track
        if not has_any_track and gap_start_s is None:
            gap_start_s = frame.timestamp_s
        if has_any_track and gap_start_s is not None:
            longest_s = max(longest_s, frame.timestamp_s - gap_start_s)
            gap_start_s = None
        last_timestamp_s = frame.timestamp_s
    if gap_start_s is not None:
        longest_s = max(longest_s, last_timestamp_s - gap_start_s)
    return float(longest_s)


def dominant_track_id(samples: list[TrackSample]) -> int | None:
    counts: dict[int, int] = {}
    for sample in samples:
        counts[int(sample.track_id)] = counts.get(int(sample.track_id), 0) + 1
    if not counts:
        return None
    return sorted(counts.items(), key=lambda item: (-item[1], item[0]))[0][0]


def clean_target_path(
    samples: list[TrackSample],
    *,
    smoothing_window: int = 5,
) -> list[CleanedPathPoint]:
    target_map_samples = [sample for sample in samples if sample.sample_kind == "target_map"]
    source_samples = target_map_samples or [
        sample for sample in samples if sample.sample_kind == "raw"
    ]
    track_id = dominant_track_id(source_samples)
    if track_id is None:
        return []

    selected = sorted(
        [sample for sample in source_samples if int(sample.track_id) == track_id],
        key=lambda sample: sample.timestamp_s,
    )
    if not selected:
        return []

    window = max(int(smoothing_window), 1)
    half_window = window // 2
    cleaned: list[CleanedPathPoint] = []
    for index, sample in enumerate(selected):
        left = max(0, index - half_window)
        right = min(len(selected), index + half_window + 1)
        positions = np.asarray(
            [item.position_m for item in selected[left:right]],
            dtype=np.float64,
        )
        cleaned_position = np.mean(positions, axis=0)
        cleaned.append(
            CleanedPathPoint(
                timestamp_s=sample.timestamp_s,
                sample_kind=sample.sample_kind,
                track_id=sample.track_id,
                source=sample.source,
                position_m=sample.position_m.copy(),
                cleaned_position_m=cleaned_position,
                confidence=sample.confidence,
                position_uncertainty_m=sample.position_uncertainty_m,
            )
        )
    return cleaned


def cleaned_path_to_csv_row(
    point: CleanedPathPoint,
    *,
    run_id: str,
    experiment_tag: str,
) -> dict[str, object]:
    return {
        "run_id": run_id,
        "experiment_tag": experiment_tag,
        "timestamp_s": f"{point.timestamp_s:.9f}",
        "sample_kind": point.sample_kind,
        "track_id": point.track_id,
        "source": point.source,
        "source_label": source_label(point.source),
        "x_m": f"{point.position_m[0]:.6f}",
        "y_m": f"{point.position_m[1]:.6f}",
        "z_m": f"{point.position_m[2]:.6f}",
        "cleaned_x_m": f"{point.cleaned_position_m[0]:.6f}",
        "cleaned_y_m": f"{point.cleaned_position_m[1]:.6f}",
        "cleaned_z_m": f"{point.cleaned_position_m[2]:.6f}",
        "confidence": f"{point.confidence:.6f}",
        "position_uncertainty_m": f"{point.position_uncertainty_m:.6f}",
    }


def simulated_dropout_backtest(
    samples: list[TrackSample],
    *,
    horizon_s: float = 1.0,
    max_velocity_seed_dt_s: float = 0.75,
) -> list[float]:
    raw_samples = [
        sample
        for sample in samples
        if sample.sample_kind == "raw" and sample.source != TRACK_SOURCE_PREDICTED
    ]
    track_id = dominant_track_id(raw_samples)
    if track_id is None:
        return []
    selected = sorted(
        [sample for sample in raw_samples if int(sample.track_id) == track_id],
        key=lambda sample: sample.timestamp_s,
    )
    if len(selected) < 3:
        return []

    horizon_s = max(float(horizon_s), 0.0)
    errors: list[float] = []
    for index in range(1, len(selected) - 1):
        previous = selected[index - 1]
        seed = selected[index]
        seed_dt_s = seed.timestamp_s - previous.timestamp_s
        if seed_dt_s <= 1e-6 or seed_dt_s > max_velocity_seed_dt_s:
            continue
        future = next(
            (
                candidate
                for candidate in selected[index + 1 :]
                if candidate.timestamp_s >= seed.timestamp_s + horizon_s
            ),
            None,
        )
        if future is None:
            continue
        velocity_mps = (seed.position_m - previous.position_m) / seed_dt_s
        predicted_position = seed.position_m + (
            velocity_mps * (future.timestamp_s - seed.timestamp_s)
        )
        error_m = float(np.linalg.norm(predicted_position - future.position_m))
        if math.isfinite(error_m):
            errors.append(error_m)
    return errors


def write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_recording_timestamp_s(name: str) -> float | None:
    match = re.search(r"_(\d{8})_(\d{6})_(\d{3})(?:_|$)", str(name))
    if match is None:
        return None
    date_text, time_text, millis_text = match.groups()
    try:
        base_s = time.mktime(time.strptime(date_text + time_text, "%Y%m%d%H%M%S"))
    except ValueError:
        return None
    return float(base_s) + (float(millis_text) / 1000.0)


def read_reference_video_info(path: Path | str) -> ReferenceVideoInfo | None:
    if cv2 is None:
        return None
    video_path = Path(path).expanduser()
    if not video_path.exists():
        return None
    capture = cv2.VideoCapture(str(video_path))
    try:
        if not capture.isOpened():
            return None
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        frame_count = int(round(float(capture.get(cv2.CAP_PROP_FRAME_COUNT))))
        width = int(round(float(capture.get(cv2.CAP_PROP_FRAME_WIDTH))))
        height = int(round(float(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))))
    finally:
        capture.release()
    if fps <= 0.0 or frame_count <= 0:
        return None
    return ReferenceVideoInfo(
        path=video_path,
        fps=fps,
        frame_count=frame_count,
        width=max(width, 0),
        height=max(height, 0),
    )


def find_nearest_reference_video(
    *,
    run_dir: Path | str,
    recording_dir: Path | str,
    run_id: str = "",
    drone_id: str = "",
) -> Path | None:
    recordings = Path(recording_dir).expanduser()
    if not recordings.exists():
        return None

    drone_id = str(drone_id or "").strip()
    if drone_id:
        candidates = list(recordings.glob(f"mission_{drone_id}_*_combined.mp4"))
        if not candidates:
            candidates = list(recordings.glob(REFERENCE_VIDEO_PATTERN))
    else:
        candidates = list(recordings.glob(REFERENCE_VIDEO_PATTERN))
    if not candidates:
        return None

    run_path = Path(run_dir)
    run_timestamp_s = parse_recording_timestamp_s(run_id or run_path.name)
    run_mtime_s = run_path.stat().st_mtime if run_path.exists() else time.time()

    def score(candidate: Path) -> tuple[int, float, str]:
        candidate_timestamp_s = parse_recording_timestamp_s(candidate.name)
        if run_timestamp_s is not None and candidate_timestamp_s is not None:
            delta_s = abs(candidate_timestamp_s - run_timestamp_s)
            if delta_s <= MAX_REFERENCE_START_DELTA_S:
                return (0, delta_s, candidate.name)
            return (2, delta_s, candidate.name)
        try:
            delta_s = abs(candidate.stat().st_mtime - run_mtime_s)
        except OSError:
            delta_s = float("inf")
        return (1, delta_s, candidate.name)

    best = sorted(candidates, key=score)[0]
    best_score = score(best)
    if best_score[0] == 2:
        return None
    return best


def resolve_reference_video_info(
    *,
    run_dir: Path | str,
    run_id: str,
    drone_id: str,
    reference_video: Path | str | None,
    reference_recording_dir: Path | str | None,
    wait_s: float,
) -> tuple[ReferenceVideoInfo | None, str]:
    deadline_s = time.monotonic() + max(float(wait_s), 0.0)
    fallback_reason = "reference_video_not_found"

    while True:
        candidate: Path | None = None
        explicit_reference = str(reference_video or "").strip()
        if explicit_reference:
            candidate = Path(explicit_reference).expanduser()
            if not candidate.exists():
                fallback_reason = f"reference_video_not_found:{candidate}"
                candidate = None
        elif reference_recording_dir is not None and str(reference_recording_dir).strip():
            candidate = find_nearest_reference_video(
                run_dir=run_dir,
                recording_dir=reference_recording_dir,
                run_id=run_id,
                drone_id=drone_id,
            )
            if candidate is None:
                fallback_reason = "reference_video_not_found"
        else:
            fallback_reason = "reference_recording_dir_not_configured"

        if candidate is not None:
            reference = read_reference_video_info(candidate)
            if reference is not None:
                return reference, ""
            fallback_reason = f"reference_video_unreadable:{candidate}"

        if time.monotonic() >= deadline_s:
            return None, fallback_reason
        time.sleep(min(0.25, max(deadline_s - time.monotonic(), 0.0)))


def cleaned_time_origin_s(
    *,
    cleaned_path: list[CleanedPathPoint],
    all_samples: list[TrackSample],
    ownship_history: list[tuple[float, np.ndarray]],
) -> tuple[float, str]:
    if cleaned_path:
        return float(cleaned_path[0].timestamp_s), "first_cleaned_path_point"
    if all_samples:
        return min(float(sample.timestamp_s) for sample in all_samples), "first_sample"
    if ownship_history:
        return min(float(timestamp_s) for timestamp_s, _ in ownship_history), "first_ownship_pose"
    return 0.0, "zero"


def _relative_time(timestamp_s: float, origin_s: float) -> float:
    value = float(timestamp_s) - float(origin_s)
    return value if value > 0.0 else 0.0


def normalized_cleaned_path(
    cleaned_path: list[CleanedPathPoint],
    *,
    origin_s: float,
) -> list[CleanedPathPoint]:
    return [
        replace(point, timestamp_s=_relative_time(point.timestamp_s, origin_s))
        for point in cleaned_path
    ]


def normalized_track_samples(
    samples: list[TrackSample],
    *,
    origin_s: float,
) -> list[TrackSample]:
    return [
        replace(sample, timestamp_s=_relative_time(sample.timestamp_s, origin_s))
        for sample in samples
    ]


def normalized_ownship_history(
    history: list[tuple[float, np.ndarray]],
    *,
    origin_s: float,
) -> list[tuple[float, np.ndarray]]:
    return [
        (_relative_time(timestamp_s, origin_s), position.copy())
        for timestamp_s, position in history
    ]


def write_cleaned_track_outputs(
    *,
    run_id: str,
    experiment_tag: str,
    drone_id: str,
    run_dir: Path,
    cleaned_path: list[CleanedPathPoint],
    all_samples: list[TrackSample],
    ownship_history: list[tuple[float, np.ndarray]],
    write_video: bool,
    video_fps: float,
    video_width: int,
    video_height: int,
    video_fourcc: str,
    reference_video: Path | str | None = None,
    reference_recording_dir: Path | str | None = None,
    reference_video_wait_s: float = 0.0,
) -> CleanedTrackPostprocessResult:
    run_dir.mkdir(parents=True, exist_ok=True)
    origin_s, origin_source = cleaned_time_origin_s(
        cleaned_path=cleaned_path,
        all_samples=all_samples,
        ownship_history=ownship_history,
    )
    relative_cleaned_path = normalized_cleaned_path(
        cleaned_path,
        origin_s=origin_s,
    )
    relative_samples = normalized_track_samples(all_samples, origin_s=origin_s)
    relative_ownship = normalized_ownship_history(
        ownship_history,
        origin_s=origin_s,
    )

    cleaned_path_path = run_dir / CLEANED_PATH_CSV
    video_path = run_dir / CLEANED_PATH_MP4
    cleaned_rows = [
        cleaned_path_to_csv_row(
            point,
            run_id=run_id,
            experiment_tag=experiment_tag,
        )
        for point in relative_cleaned_path
    ]
    write_csv(cleaned_path_path, CLEANED_PATH_FIELDNAMES, cleaned_rows)

    reference: ReferenceVideoInfo | None = None
    fallback_reason = "video_disabled"
    if write_video:
        reference, fallback_reason = resolve_reference_video_info(
            run_dir=run_dir,
            run_id=run_id,
            drone_id=drone_id,
            reference_video=reference_video,
            reference_recording_dir=reference_recording_dir,
            wait_s=reference_video_wait_s,
        )
    render_fps = max(float(video_fps), 1.0)
    render_frame_count: int | None = None
    video_synced = False
    if reference is not None:
        render_fps = reference.fps
        render_frame_count = reference.frame_count
        video_synced = True

    cleaned_path_mp4: Path | None = None
    actual_frame_count = 0
    if write_video:
        wrote_video = write_cleaned_path_video(
            video_path,
            cleaned_path=relative_cleaned_path,
            all_samples=relative_samples,
            ownship_history=relative_ownship,
            fps=render_fps,
            width=int(video_width),
            height=int(video_height),
            fourcc_text=video_fourcc,
            frame_count=render_frame_count,
        )
        if wrote_video:
            cleaned_path_mp4 = video_path
            actual_frame_count = (
                render_frame_count
                if render_frame_count is not None
                else len(relative_cleaned_path)
            )
        elif video_synced:
            fallback_reason = "video_write_failed"

    return CleanedTrackPostprocessResult(
        cleaned_path_csv=cleaned_path_path,
        cleaned_path_mp4=cleaned_path_mp4,
        time_origin_s=origin_s,
        time_origin_source=origin_source,
        reference_video=reference,
        video_synced_to_reference=video_synced and cleaned_path_mp4 is not None,
        video_fps=render_fps if cleaned_path_mp4 is not None else 0.0,
        video_frame_count=actual_frame_count,
        fallback_reason=(
            "" if video_synced and cleaned_path_mp4 is not None else fallback_reason
        ),
    )


def write_run_outputs(
    run: TrackingMetricsRun,
    *,
    run_dir: Path,
    drone_id: str = "",
    write_video: bool = True,
    video_fps: float = 10.0,
    video_width: int = 1280,
    video_height: int = 720,
    video_fourcc: str = "mp4v",
    postprocess_cleaned_tracks: bool = True,
    reference_video: Path | str | None = None,
    reference_recording_dir: Path | str | None = None,
    reference_video_wait_s: float = 0.0,
) -> dict[str, str]:
    run_dir.mkdir(parents=True, exist_ok=True)
    sample_rows = [
        sample_to_csv_row(
            sample,
            run_id=run.run_id,
            experiment_tag=run.experiment_tag,
        )
        for sample in sorted(run.samples, key=lambda item: item.timestamp_s)
    ]
    frame_rows = [
        frame_to_csv_row(
            frame,
            run_id=run.run_id,
            experiment_tag=run.experiment_tag,
        )
        for frame in run.frames
    ]
    summary = run.summary()

    samples_path = run_dir / SAMPLES_CSV
    frames_path = run_dir / FRAMES_CSV
    summary_path = run_dir / SUMMARY_CSV
    manifest_path = run_dir / MANIFEST_JSON

    write_csv(samples_path, SAMPLE_FIELDNAMES, sample_rows)
    write_csv(frames_path, FRAME_FIELDNAMES, frame_rows)
    write_csv(summary_path, SUMMARY_FIELDNAMES, [summary])

    artifacts = {
        "samples_csv": str(samples_path),
        "frames_csv": str(frames_path),
        "cleaned_path_csv": "",
        "summary_csv": str(summary_path),
        "manifest_json": str(manifest_path),
        "cleaned_path_mp4": "",
    }
    postprocess_manifest: dict[str, object] = {
        "enabled": bool(postprocess_cleaned_tracks),
    }
    if postprocess_cleaned_tracks:
        postprocess_result = write_cleaned_track_outputs(
            run_id=run.run_id,
            experiment_tag=run.experiment_tag,
            drone_id=drone_id,
            run_dir=run_dir,
            cleaned_path=run.cleaned_path(),
            all_samples=run.samples,
            ownship_history=run.ownship_history,
            write_video=write_video,
            video_fps=video_fps,
            video_width=video_width,
            video_height=video_height,
            video_fourcc=video_fourcc,
            reference_video=reference_video,
            reference_recording_dir=reference_recording_dir,
            reference_video_wait_s=reference_video_wait_s,
        )
        postprocess_manifest = postprocess_result.to_manifest_dict()
        postprocess_manifest["enabled"] = True
        artifacts["cleaned_path_csv"] = str(postprocess_result.cleaned_path_csv)
        if postprocess_result.cleaned_path_mp4 is not None:
            artifacts["cleaned_path_mp4"] = str(postprocess_result.cleaned_path_mp4)

    manifest = {
        "run_id": run.run_id,
        "experiment_tag": run.experiment_tag,
        "created_at_wall": time.strftime(
            "%Y-%m-%dT%H:%M:%S%z",
            time.localtime(time.time()),
        ),
        "files": artifacts,
        "summary": summary,
        "postprocessing": postprocess_manifest,
    }
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return artifacts


def _history_start_s(frame_time_s: float, history_window_s: float) -> float:
    return max(float(frame_time_s) - max(float(history_window_s), 0.0), 0.0)


def _timestamp_in_history_window(
    timestamp_s: float,
    *,
    frame_time_s: float,
    history_window_s: float,
) -> bool:
    return (
        _history_start_s(frame_time_s, history_window_s)
        <= float(timestamp_s)
        <= float(frame_time_s)
    )


def samples_in_history_window(
    samples: list[TrackSample],
    *,
    frame_time_s: float,
    history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S,
) -> list[TrackSample]:
    return [
        sample
        for sample in samples
        if _timestamp_in_history_window(
            sample.timestamp_s,
            frame_time_s=frame_time_s,
            history_window_s=history_window_s,
        )
    ]


def positions_in_history_window(
    history: list[tuple[float, np.ndarray]],
    *,
    frame_time_s: float,
    history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S,
) -> list[np.ndarray]:
    return [
        position
        for timestamp_s, position in history
        if _timestamp_in_history_window(
            timestamp_s,
            frame_time_s=frame_time_s,
            history_window_s=history_window_s,
        )
    ]


def cleaned_points_in_history_window(
    cleaned_path: list[CleanedPathPoint],
    *,
    frame_time_s: float,
    history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S,
) -> list[CleanedPathPoint]:
    return [
        point
        for point in cleaned_path
        if _timestamp_in_history_window(
            point.timestamp_s,
            frame_time_s=frame_time_s,
            history_window_s=history_window_s,
        )
    ]


def write_cleaned_path_video(
    output_path: Path,
    *,
    cleaned_path: list[CleanedPathPoint],
    all_samples: list[TrackSample],
    ownship_history: list[tuple[float, np.ndarray]],
    fps: float = 10.0,
    width: int = 1280,
    height: int = 720,
    fourcc_text: str = "mp4v",
    frame_count: int | None = None,
    history_window_s: float = CLEANED_VIDEO_HISTORY_WINDOW_S,
) -> bool:
    if cv2 is None or not cleaned_path:
        return False

    output_frame_count = (
        max(int(frame_count), 0) if frame_count is not None else len(cleaned_path)
    )
    if output_frame_count <= 0:
        return False

    width = max(int(width), 640)
    height = max(int(height), 480)
    if width % 2:
        width += 1
    if height % 2:
        height += 1
    output_path.parent.mkdir(parents=True, exist_ok=True)
    render_fps = max(float(fps), 1.0)
    fourcc = cv2.VideoWriter_fourcc(*(str(fourcc_text or "mp4v")[:4].ljust(4)))
    writer = cv2.VideoWriter(str(output_path), fourcc, render_fps, (width, height))
    if not writer.isOpened():
        writer.release()
        return False

    all_points = [point.cleaned_position_m for point in cleaned_path]
    all_points.extend(sample.position_m for sample in all_samples)
    all_points.extend(position for _, position in ownship_history)
    bounds = map_bounds(all_points)
    raw_samples = sorted(
        [sample for sample in all_samples if sample.sample_kind == "raw"],
        key=lambda sample: sample.timestamp_s,
    )
    map_samples = sorted(
        [sample for sample in all_samples if sample.sample_kind == "target_map"],
        key=lambda sample: sample.timestamp_s,
    )

    path_index = -1
    for frame_index in range(output_frame_count):
        if frame_count is None:
            path_index = frame_index
            frame_time_s = float(cleaned_path[path_index].timestamp_s)
        else:
            frame_time_s = float(frame_index) / render_fps
            while (
                path_index + 1 < len(cleaned_path)
                and cleaned_path[path_index + 1].timestamp_s <= frame_time_s + 1e-9
            ):
                path_index += 1

        current_available = path_index >= 0
        point = cleaned_path[path_index] if current_available else cleaned_path[0]
        point_visible = (
            current_available
            and _timestamp_in_history_window(
                point.timestamp_s,
                frame_time_s=frame_time_s,
                history_window_s=history_window_s,
            )
        )
        visible_cleaned_path = (
            cleaned_points_in_history_window(
                cleaned_path[: path_index + 1],
                frame_time_s=frame_time_s,
                history_window_s=history_window_s,
            )
            if current_available
            else []
        )
        recent_map_samples = samples_in_history_window(
            map_samples,
            frame_time_s=frame_time_s,
            history_window_s=history_window_s,
        )

        frame = np.full((height, width, 3), (18, 20, 24), dtype=np.uint8)
        draw_grid(frame, bounds)
        draw_history_polyline(
            frame,
            positions_in_history_window(
                ownship_history,
                frame_time_s=frame_time_s,
                history_window_s=history_window_s,
            ),
            bounds,
            color=(255, 150, 40),
            thickness=2,
        )
        draw_sample_points(
            frame,
            samples_in_history_window(
                raw_samples,
                frame_time_s=frame_time_s,
                history_window_s=history_window_s,
            ),
            bounds,
            color=(80, 220, 80),
            radius=3,
        )
        draw_sample_points(
            frame,
            [
                sample
                for sample in recent_map_samples
                if sample.source != TRACK_SOURCE_PREDICTED
            ],
            bounds,
            color=(0, 210, 255),
            radius=3,
        )
        draw_sample_points(
            frame,
            [
                sample
                for sample in recent_map_samples
                if sample.source == TRACK_SOURCE_PREDICTED
            ],
            bounds,
            color=(0, 120, 255),
            radius=3,
        )
        draw_history_polyline(
            frame,
            [item.cleaned_position_m for item in visible_cleaned_path],
            bounds,
            color=(255, 255, 255),
            thickness=3,
        )
        if point_visible:
            current_px = world_to_pixel(point.cleaned_position_m, bounds, width, height)
            uncertainty_px = world_radius_to_px(
                point.position_uncertainty_m,
                bounds,
                width,
                height,
            )
            if uncertainty_px > 1:
                cv2.circle(
                    frame,
                    current_px,
                    uncertainty_px,
                    (0, 120, 255),
                    1,
                    cv2.LINE_AA,
                )
            cv2.circle(frame, current_px, 7, (255, 255, 255), -1, cv2.LINE_AA)
            cv2.circle(frame, current_px, 9, (0, 0, 0), 1, cv2.LINE_AA)
        draw_video_overlay(
            frame,
            point=point if point_visible else None,
            index=frame_index,
            total=output_frame_count,
            display_time_s=frame_time_s,
        )
        writer.write(frame)

    writer.release()
    return True


def map_bounds(points: list[np.ndarray]) -> tuple[float, float, float, float]:
    if not points:
        return -2.0, 2.0, -2.0, 2.0
    array = np.asarray(points, dtype=np.float64)
    xs = array[:, 0]
    ys = array[:, 1]
    min_x = float(np.min(xs))
    max_x = float(np.max(xs))
    min_y = float(np.min(ys))
    max_y = float(np.max(ys))
    margin = max(max_x - min_x, max_y - min_y, 1.0) * 0.15
    return min_x - margin, max_x + margin, min_y - margin, max_y + margin


def world_to_pixel(
    position_m: np.ndarray,
    bounds: tuple[float, float, float, float],
    width: int,
    height: int,
) -> tuple[int, int]:
    min_x, max_x, min_y, max_y = bounds
    map_w = max(max_x - min_x, 1e-6)
    map_h = max(max_y - min_y, 1e-6)
    pad = 54
    px = int(round(pad + ((float(position_m[0]) - min_x) / map_w) * (width - 2 * pad)))
    py = int(round(height - pad - ((float(position_m[1]) - min_y) / map_h) * (height - 2 * pad)))
    return px, py


def world_radius_to_px(
    radius_m: float,
    bounds: tuple[float, float, float, float],
    width: int,
    height: int,
) -> int:
    min_x, max_x, min_y, max_y = bounds
    scale_x = (width - 108) / max(max_x - min_x, 1e-6)
    scale_y = (height - 108) / max(max_y - min_y, 1e-6)
    return int(round(max(radius_m, 0.0) * min(scale_x, scale_y)))


def draw_grid(frame: np.ndarray, bounds: tuple[float, float, float, float]) -> None:
    if cv2 is None:
        return
    height, width = frame.shape[:2]
    min_x, max_x, min_y, max_y = bounds
    for x_value in np.linspace(min_x, max_x, 9):
        start = world_to_pixel(np.array([x_value, min_y, 0.0]), bounds, width, height)
        end = world_to_pixel(np.array([x_value, max_y, 0.0]), bounds, width, height)
        cv2.line(frame, start, end, (55, 58, 66), 1, cv2.LINE_AA)
    for y_value in np.linspace(min_y, max_y, 7):
        start = world_to_pixel(np.array([min_x, y_value, 0.0]), bounds, width, height)
        end = world_to_pixel(np.array([max_x, y_value, 0.0]), bounds, width, height)
        cv2.line(frame, start, end, (55, 58, 66), 1, cv2.LINE_AA)


def draw_history_polyline(
    frame: np.ndarray,
    positions: list[np.ndarray],
    bounds: tuple[float, float, float, float],
    *,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    if cv2 is None or len(positions) < 2:
        return
    height, width = frame.shape[:2]
    points = [world_to_pixel(position, bounds, width, height) for position in positions]
    for start, end in zip(points[:-1], points[1:]):
        cv2.line(frame, start, end, color, thickness, cv2.LINE_AA)


def draw_sample_points(
    frame: np.ndarray,
    samples: list[TrackSample],
    bounds: tuple[float, float, float, float],
    *,
    color: tuple[int, int, int],
    radius: int,
) -> None:
    if cv2 is None:
        return
    height, width = frame.shape[:2]
    for sample in samples[-350:]:
        point_px = world_to_pixel(sample.position_m, bounds, width, height)
        cv2.circle(frame, point_px, radius, color, -1, cv2.LINE_AA)


def draw_video_overlay(
    frame: np.ndarray,
    *,
    point: CleanedPathPoint | None,
    index: int,
    total: int,
    display_time_s: float,
) -> None:
    if cv2 is None:
        return
    if point is None:
        track_line = "waiting for cleaned target"
    else:
        track_line = (
            f"source={point.sample_kind}/{source_label(point.source)}  "
            f"track_id={point.track_id}  unc={point.position_uncertainty_m:.2f}m"
        )
    lines = [
        "Milestone tracking metrics: cleaned estimated target path",
        f"frame {index + 1}/{total}  t={display_time_s:.2f}s",
        track_line,
        "blue=ownship  green=raw visual  yellow=target-map observed  orange=predicted  white=cleaned",
    ]
    cv2.rectangle(frame, (18, 18), (frame.shape[1] - 18, 132), (0, 0, 0), -1)
    cv2.rectangle(frame, (18, 18), (frame.shape[1] - 18, 132), (210, 210, 210), 1)
    y = 45
    for line in lines:
        cv2.putText(
            frame,
            line,
            (32, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.64,
            (245, 245, 245),
            2,
            cv2.LINE_AA,
        )
        y += 25


class TrackingMetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("tracking_metrics_node")
        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("experiment_tag", "visual_metrics")
        self.declare_parameter("output_dir", "mission_recordings/tracking_metrics")
        self.declare_parameter("raw_world_tracks_topic", "")
        self.declare_parameter("target_map_world_tracks_topic", "")
        self.declare_parameter("ownship_pose_topic", configured_ownship_pose_topic())
        self.declare_parameter("engagement_state_topic", "")
        self.declare_parameter("metrics_rate_hz", 10.0)
        self.declare_parameter("track_stale_s", 0.5)
        self.declare_parameter("max_reacquisition_gap_s", 3.0)
        self.declare_parameter("simulated_dropout_horizon_s", 1.0)
        self.declare_parameter("path_smoothing_window", 5)
        self.declare_parameter("write_video", True)
        self.declare_parameter("video_fps", 10.0)
        self.declare_parameter("video_width", 1280)
        self.declare_parameter("video_height", 720)
        self.declare_parameter("video_fourcc", "mp4v")
        self.declare_parameter("postprocess_cleaned_tracks", True)
        self.declare_parameter("reference_video", "")
        self.declare_parameter("reference_recording_dir", "mission_recordings")
        self.declare_parameter("reference_video_wait_s", 5.0)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.experiment_tag = str(self.get_parameter("experiment_tag").value).strip()
        self.output_dir = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.raw_world_tracks_topic = (
            str(self.get_parameter("raw_world_tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/world_tracks")
        )
        target_map_topic = str(
            self.get_parameter("target_map_world_tracks_topic").value
        ).strip()
        self.target_map_world_tracks_topic = target_map_topic
        self.ownship_pose_topic = (
            str(self.get_parameter("ownship_pose_topic").value).strip()
            or configured_ownship_pose_topic()
        )
        self.engagement_state_topic = (
            str(self.get_parameter("engagement_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "engagement/state")
        )
        self.metrics_rate_hz = float(self.get_parameter("metrics_rate_hz").value)
        self.track_stale_s = float(self.get_parameter("track_stale_s").value)
        self.max_reacquisition_gap_s = float(
            self.get_parameter("max_reacquisition_gap_s").value
        )
        self.simulated_dropout_horizon_s = float(
            self.get_parameter("simulated_dropout_horizon_s").value
        )
        self.path_smoothing_window = int(
            self.get_parameter("path_smoothing_window").value
        )
        self.write_video = bool(self.get_parameter("write_video").value)
        self.video_fps = float(self.get_parameter("video_fps").value)
        self.video_width = int(self.get_parameter("video_width").value)
        self.video_height = int(self.get_parameter("video_height").value)
        self.video_fourcc = str(self.get_parameter("video_fourcc").value)
        self.postprocess_cleaned_tracks = bool(
            self.get_parameter("postprocess_cleaned_tracks").value
        )
        self.reference_video = str(self.get_parameter("reference_video").value).strip()
        self.reference_recording_dir = Path(
            str(self.get_parameter("reference_recording_dir").value).strip()
            or "mission_recordings"
        ).expanduser()
        self.reference_video_wait_s = float(
            self.get_parameter("reference_video_wait_s").value
        )

        self.current_run: TrackingMetricsRun | None = None
        self.current_run_dir: Path | None = None
        self.latest_raw_tracks: list[TrackSample] = []
        self.latest_map_tracks: list[TrackSample] = []
        self.last_raw_tracks_s = -1.0
        self.last_map_tracks_s = -1.0
        self.latest_state = "IDLE"
        self.last_finalize_wall_s = 0.0

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            WorldTargetTrackArray,
            self.raw_world_tracks_topic,
            self.raw_tracks_callback,
            10,
        )
        if self.target_map_world_tracks_topic:
            self.create_subscription(
                WorldTargetTrackArray,
                self.target_map_world_tracks_topic,
                self.target_map_tracks_callback,
                10,
            )
        self.create_subscription(
            PoseStamped,
            self.ownship_pose_topic,
            self.ownship_pose_callback,
            best_effort_qos,
        )
        self.create_subscription(
            EngagementState,
            self.engagement_state_topic,
            self.engagement_state_callback,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(self.metrics_rate_hz, 1.0),
            self.timer_callback,
        )
        self.get_logger().info(
            "Tracking metrics started: "
            f"raw={self.raw_world_tracks_topic}, "
            f"target_map={self.target_map_world_tracks_topic or 'disabled'}, "
            f"ownship={self.ownship_pose_topic}, state={self.engagement_state_topic}, "
            f"output_dir={self.output_dir}, "
            f"postprocess_cleaned_tracks={self.postprocess_cleaned_tracks}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def raw_tracks_callback(self, msg: WorldTargetTrackArray) -> None:
        timestamp_s = self._message_time_s(msg)
        tracks = [
            sample_from_world_track(track, timestamp_s=timestamp_s, sample_kind="raw")
            for track in msg.tracks
        ]
        self.latest_raw_tracks = tracks
        self.last_raw_tracks_s = self.now_s()
        if self.current_run is not None:
            self.current_run.add_samples(tracks)

    def target_map_tracks_callback(self, msg: WorldTargetTrackArray) -> None:
        timestamp_s = self._message_time_s(msg)
        tracks = [
            sample_from_world_track(
                track,
                timestamp_s=timestamp_s,
                sample_kind="target_map",
            )
            for track in msg.tracks
        ]
        self.latest_map_tracks = tracks
        self.last_map_tracks_s = self.now_s()
        if self.current_run is not None:
            self.current_run.add_samples(tracks)

    def ownship_pose_callback(self, msg: PoseStamped) -> None:
        if self.current_run is None:
            return
        timestamp_s = self._stamp_or_now_s(msg.header.stamp)
        self.current_run.add_ownship_position(
            timestamp_s,
            np.array(
                [
                    finite_float(msg.pose.position.x),
                    finite_float(msg.pose.position.y),
                    finite_float(msg.pose.position.z),
                ],
                dtype=np.float64,
            ),
        )

    def engagement_state_callback(self, msg: EngagementState) -> None:
        self.latest_state = str(msg.state or "")
        if is_active_mission_state(self.latest_state):
            self._start_run_if_needed()
        elif self.current_run is not None:
            self._finalize_current_run(reason=self.latest_state or "terminal")

    def timer_callback(self) -> None:
        run = self.current_run
        if run is None:
            return
        now_s = self.now_s()
        raw_tracks = (
            self.latest_raw_tracks
            if self.last_raw_tracks_s >= 0.0
            and now_s - self.last_raw_tracks_s <= self.track_stale_s
            else []
        )
        map_tracks = (
            self.latest_map_tracks
            if self.last_map_tracks_s >= 0.0
            and now_s - self.last_map_tracks_s <= self.track_stale_s
            else []
        )
        run.sample_frame(
            timestamp_s=now_s,
            state=self.latest_state,
            raw_tracks=raw_tracks,
            target_map_tracks=map_tracks,
        )

    def destroy_node(self) -> bool:
        if self.current_run is not None:
            self._finalize_current_run(reason="node_shutdown")
        return super().destroy_node()

    def _message_time_s(self, msg: WorldTargetTrackArray) -> float:
        timestamp_s = stamp_to_s(msg.stamp)
        return timestamp_s if timestamp_s > 0.0 else self.now_s()

    def _stamp_or_now_s(self, stamp: Any) -> float:
        timestamp_s = stamp_to_s(stamp)
        return timestamp_s if timestamp_s > 0.0 else self.now_s()

    def _start_run_if_needed(self) -> None:
        if self.current_run is not None:
            return
        started_wall_s = time.time()
        tag = self.experiment_tag or "visual_metrics"
        run_id = make_run_id(
            drone_id=self.drone_id,
            experiment_tag=tag,
            started_wall_time_s=started_wall_s,
        )
        self.current_run = TrackingMetricsRun(
            run_id=run_id,
            experiment_tag=tag,
            max_reacquisition_gap_s=self.max_reacquisition_gap_s,
            simulated_dropout_horizon_s=self.simulated_dropout_horizon_s,
            smoothing_window=self.path_smoothing_window,
        )
        self.current_run_dir = self.output_dir / run_id
        self.get_logger().info(f"Tracking metrics run started: {self.current_run_dir}")

    def _finalize_current_run(self, *, reason: str) -> None:
        run = self.current_run
        run_dir = self.current_run_dir
        if run is None or run_dir is None:
            return
        self.current_run = None
        self.current_run_dir = None
        artifacts = write_run_outputs(
            run,
            run_dir=run_dir,
            drone_id=self.drone_id,
            write_video=self.write_video,
            video_fps=self.video_fps,
            video_width=self.video_width,
            video_height=self.video_height,
            video_fourcc=self.video_fourcc,
            postprocess_cleaned_tracks=self.postprocess_cleaned_tracks,
            reference_video=self.reference_video,
            reference_recording_dir=self.reference_recording_dir,
            reference_video_wait_s=self.reference_video_wait_s,
        )
        self.last_finalize_wall_s = time.time()
        self.get_logger().info(
            "Tracking metrics run finalized "
            f"({reason}): "
            + ", ".join(f"{name}={path}" for name, path in artifacts.items() if path)
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackingMetricsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _csv_int(value: Any, default: int = 0) -> int:
    try:
        return int(float(str(value).strip()))
    except (TypeError, ValueError):
        return default


def _csv_bool(value: Any) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _drone_id_from_run_id(run_id: str) -> str:
    match = re.match(r"^tracking_(.+)_\d{8}_\d{6}_\d{3}(?:_|$)", str(run_id))
    return "" if match is None else match.group(1)


def _default_reference_recording_dir(run_dir: Path) -> Path:
    if run_dir.parent.name == "tracking_metrics":
        return run_dir.parent.parent
    return Path("mission_recordings")


def read_tracking_samples_csv(run_dir: Path | str) -> tuple[list[TrackSample], str, str]:
    run_path = Path(run_dir).expanduser()
    samples_path = run_path / SAMPLES_CSV
    if not samples_path.exists():
        raise FileNotFoundError(f"tracking samples not found: {samples_path}")

    samples: list[TrackSample] = []
    run_id = ""
    experiment_tag = ""
    with samples_path.open("r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            if not run_id:
                run_id = str(row.get("run_id") or "").strip()
            if not experiment_tag:
                experiment_tag = str(row.get("experiment_tag") or "").strip()
            track_id = _csv_int(row.get("track_id"), 0)
            detector_track_id = _csv_int(
                row.get("detector_track_id"),
                track_id,
            )
            samples.append(
                TrackSample(
                    timestamp_s=finite_float(row.get("timestamp_s")),
                    sample_kind=str(row.get("sample_kind") or ""),
                    track_id=track_id,
                    detector_track_id=detector_track_id,
                    source=_csv_int(row.get("source"), TRACK_SOURCE_DETECTED),
                    position_m=np.array(
                        [
                            finite_float(row.get("x_m")),
                            finite_float(row.get("y_m")),
                            finite_float(row.get("z_m")),
                        ],
                        dtype=np.float64,
                    ),
                    velocity_mps=np.array(
                        [
                            finite_float(row.get("vx_mps")),
                            finite_float(row.get("vy_mps")),
                            finite_float(row.get("vz_mps")),
                        ],
                        dtype=np.float64,
                    ),
                    confidence=finite_float(row.get("confidence")),
                    distance_m=finite_float(row.get("distance_m")),
                    bbox_area_px=finite_float(row.get("bbox_area_px")),
                    inbound=_csv_bool(row.get("inbound")),
                    last_observed_age_s=finite_float(row.get("last_observed_age_s")),
                    prediction_horizon_s=finite_float(row.get("prediction_horizon_s")),
                    position_uncertainty_m=finite_float(
                        row.get("position_uncertainty_m")
                    ),
                    velocity_uncertainty_mps=finite_float(
                        row.get("velocity_uncertainty_mps")
                    ),
                )
            )

    if not run_id:
        run_id = run_path.name
    if not experiment_tag:
        experiment_tag = "visual_metrics"
    return samples, run_id, experiment_tag


def update_tracking_manifest(
    *,
    run_dir: Path,
    run_id: str,
    experiment_tag: str,
    result: CleanedTrackPostprocessResult,
) -> Path:
    manifest_path = run_dir / MANIFEST_JSON
    manifest: dict[str, object] = {}
    if manifest_path.exists():
        try:
            loaded = json.loads(manifest_path.read_text(encoding="utf-8"))
            if isinstance(loaded, dict):
                manifest = loaded
        except (OSError, json.JSONDecodeError):
            manifest = {}

    files = manifest.get("files")
    if not isinstance(files, dict):
        files = {}
    files.update(
        {
            "cleaned_path_csv": str(result.cleaned_path_csv),
            "cleaned_path_mp4": (
                "" if result.cleaned_path_mp4 is None else str(result.cleaned_path_mp4)
            ),
            "manifest_json": str(manifest_path),
        }
    )
    manifest.update(
        {
            "run_id": str(manifest.get("run_id") or run_id),
            "experiment_tag": str(manifest.get("experiment_tag") or experiment_tag),
            "updated_at_wall": time.strftime(
                "%Y-%m-%dT%H:%M:%S%z",
                time.localtime(time.time()),
            ),
            "files": files,
            "postprocessing": result.to_manifest_dict(),
        }
    )
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def postprocess_tracking_run_dir(
    *,
    run_dir: Path | str,
    reference_video: Path | str | None = None,
    reference_recording_dir: Path | str | None = None,
    reference_video_wait_s: float = 0.0,
    smoothing_window: int = 5,
    write_video: bool = True,
    video_fps: float = 10.0,
    video_width: int = 1280,
    video_height: int = 720,
    video_fourcc: str = "mp4v",
) -> dict[str, object]:
    run_path = Path(run_dir).expanduser()
    samples, run_id, experiment_tag = read_tracking_samples_csv(run_path)
    if reference_recording_dir is None or not str(reference_recording_dir).strip():
        reference_recording_dir = _default_reference_recording_dir(run_path)
    cleaned_path = clean_target_path(
        samples,
        smoothing_window=max(int(smoothing_window), 1),
    )
    result = write_cleaned_track_outputs(
        run_id=run_id,
        experiment_tag=experiment_tag,
        drone_id=_drone_id_from_run_id(run_id),
        run_dir=run_path,
        cleaned_path=cleaned_path,
        all_samples=samples,
        ownship_history=[],
        write_video=write_video,
        video_fps=video_fps,
        video_width=video_width,
        video_height=video_height,
        video_fourcc=video_fourcc,
        reference_video=reference_video,
        reference_recording_dir=reference_recording_dir,
        reference_video_wait_s=reference_video_wait_s,
    )
    manifest_path = update_tracking_manifest(
        run_dir=run_path,
        run_id=run_id,
        experiment_tag=experiment_tag,
        result=result,
    )
    return {
        "cleaned_path_csv": str(result.cleaned_path_csv),
        "cleaned_path_mp4": "" if result.cleaned_path_mp4 is None else str(result.cleaned_path_mp4),
        "manifest_json": str(manifest_path),
        "video_synced_to_reference": result.video_synced_to_reference,
        "reference_video_path": (
            "" if result.reference_video is None else str(result.reference_video.path)
        ),
        "fallback_reason": result.fallback_reason,
    }


def build_postprocess_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Regenerate cleaned target path outputs for a tracking run.",
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--reference-video", default="")
    parser.add_argument("--reference-recording-dir", default="")
    parser.add_argument("--reference-video-wait-s", type=float, default=0.0)
    parser.add_argument("--smoothing-window", type=int, default=5)
    parser.add_argument("--video-fps", type=float, default=10.0)
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fourcc", default="mp4v")
    parser.add_argument("--no-video", action="store_true")
    return parser


def postprocess_main(argv: list[str] | None = None) -> int:
    parser = build_postprocess_arg_parser()
    args = parser.parse_args(argv)
    artifacts = postprocess_tracking_run_dir(
        run_dir=args.run_dir,
        reference_video=args.reference_video,
        reference_recording_dir=args.reference_recording_dir,
        reference_video_wait_s=args.reference_video_wait_s,
        smoothing_window=args.smoothing_window,
        write_video=not args.no_video,
        video_fps=args.video_fps,
        video_width=args.video_width,
        video_height=args.video_height,
        video_fourcc=args.video_fourcc,
    )
    print(f"Wrote cleaned target path: {artifacts['cleaned_path_csv']}")
    if artifacts.get("cleaned_path_mp4"):
        print(f"Wrote cleaned target video: {artifacts['cleaned_path_mp4']}")
    if artifacts.get("reference_video_path"):
        print(f"Reference video: {artifacts['reference_video_path']}")
    if artifacts.get("fallback_reason"):
        print(f"Postprocess fallback: {artifacts['fallback_reason']}")
    return 0


def read_summary_csv(run_dir: Path) -> dict[str, str]:
    path = run_dir / SUMMARY_CSV
    if not path.exists():
        raise FileNotFoundError(f"tracking summary not found: {path}")
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise ValueError(f"tracking summary is empty: {path}")
    return {key: value for key, value in rows[0].items() if key is not None}


def compare_tracking_runs(
    *,
    milestone3_run_dir: str | Path,
    milestone4_run_dir: str | Path,
    output_dir: str | Path,
) -> dict[str, object]:
    m3_dir = Path(milestone3_run_dir).expanduser().resolve()
    m4_dir = Path(milestone4_run_dir).expanduser().resolve()
    out_dir = Path(output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    m3 = read_summary_csv(m3_dir)
    m4 = read_summary_csv(m4_dir)
    metrics = [
        "raw_visual_track_coverage_ratio",
        "target_map_coverage_ratio",
        "predicted_coverage_during_raw_dropout_ratio",
        "longest_raw_dropout_s",
        "longest_no_track_gap_s",
        "longest_target_map_bridge_s",
        "raw_id_switch_count",
        "target_map_id_switch_count",
        "target_map_same_id_reacquisition_ratio",
        "reacquisition_residual_median_m",
        "simulated_dropout_error_median_m",
        "uncertainty_calibration_ratio",
    ]
    rows: list[dict[str, object]] = []
    for metric in metrics:
        m3_value = optional_float(m3.get(metric))
        m4_value = optional_float(m4.get(metric))
        delta = None if m3_value is None or m4_value is None else m4_value - m3_value
        rows.append(
            {
                "metric": metric,
                "milestone3": "" if m3_value is None else f"{m3_value:.6f}",
                "milestone4": "" if m4_value is None else f"{m4_value:.6f}",
                "delta_m4_minus_m3": "" if delta is None else f"{delta:.6f}",
            }
        )

    csv_path = out_dir / "comparison_summary.csv"
    write_csv(
        csv_path,
        ["metric", "milestone3", "milestone4", "delta_m4_minus_m3"],
        rows,
    )
    report = {
        "milestone3_run_dir": str(m3_dir),
        "milestone4_run_dir": str(m4_dir),
        "comparison_csv": str(csv_path),
        "milestone3_summary": m3,
        "milestone4_summary": m4,
        "metrics": rows,
    }
    json_path = out_dir / "comparison_summary.json"
    md_path = out_dir / "comparison.md"
    json_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    write_comparison_markdown(report, md_path)
    report["comparison_json"] = str(json_path)
    report["comparison_markdown"] = str(md_path)
    return report


def write_comparison_markdown(report: dict[str, object], path: Path) -> None:
    rows = report.get("metrics", [])
    lines = [
        "# Milestone Tracking Comparison",
        "",
        f"- Milestone 3 run: `{report.get('milestone3_run_dir', '')}`",
        f"- Milestone 4 run: `{report.get('milestone4_run_dir', '')}`",
        "",
        "| Metric | Milestone 3 | Milestone 4 | Delta |",
        "| --- | ---: | ---: | ---: |",
    ]
    if isinstance(rows, list):
        for row in rows:
            if not isinstance(row, dict):
                continue
            lines.append(
                "| "
                f"{row.get('metric', '')} | "
                f"{row.get('milestone3', '')} | "
                f"{row.get('milestone4', '')} | "
                f"{row.get('delta_m4_minus_m3', '')} |"
            )
    lines.extend(
        [
            "",
            "## Metric Definitions",
            "",
            "| Metric | Meaning | Better Direction |",
            "| --- | --- | --- |",
            "| raw_visual_track_coverage_ratio | Fraction of logged frames where the raw vision detector/tracker had a visible target track. This is Milestone 3's main tracking signal. | Higher |",
            "| target_map_coverage_ratio | Fraction of logged frames where the internal target map had an active target estimate, either observed or predicted. | Higher |",
            "| predicted_coverage_during_raw_dropout_ratio | During raw visual dropouts, fraction of frames where Milestone 4 still published a predicted target-map estimate. | Higher |",
            "| longest_raw_dropout_s | Longest continuous time span where the raw visual tracker had no target. This describes detector visibility, not target-map performance. | Lower |",
            "| longest_no_track_gap_s | Longest continuous time span where the evaluated tracking source had no usable target estimate. For Milestone 3 this is raw-only; for Milestone 4 this includes target-map prediction. | Lower |",
            "| longest_target_map_bridge_s | Longest continuous raw-detector dropout bridged by target-map prediction. | Higher, within configured hold/uncertainty limits |",
            "| raw_id_switch_count | Number of times the raw detector/tracker changed active track ID during the run. | Lower |",
            "| target_map_id_switch_count | Number of times the target-map active ID changed during the run. In Milestone 4 this can expose association instability or multiple target-map instances. | Lower |",
            "| target_map_same_id_reacquisition_ratio | Fraction of raw reacquisition events where the target map kept the same internal ID across the dropout. | Higher |",
            "| reacquisition_residual_median_m | Median distance between the target-map prediction immediately before reacquisition and the first raw observed target position after reacquisition. Only available when target-map prediction is active. | Lower |",
            "| simulated_dropout_error_median_m | Offline backtest error: visible observations are temporarily hidden, then the predicted position is compared with later visible observations. | Lower |",
            "| uncertainty_calibration_ratio | Fraction of reacquisition residuals that landed inside the predicted uncertainty radius. A low value means uncertainty is overconfident; a very high value can mean uncertainty is overly conservative. | Closer to the desired confidence level |",
            "",
            "Milestone 4 is expected to improve target-map coverage, reduce no-track "
            "gaps, and bridge raw detector dropouts with predicted target-map samples.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_compare_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare Milestone 3 and Milestone 4 visual tracking metrics.",
    )
    parser.add_argument("--milestone3-run-dir", required=True)
    parser.add_argument("--milestone4-run-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    return parser


def compare_main(argv: list[str] | None = None) -> int:
    parser = build_compare_arg_parser()
    args = parser.parse_args(argv)
    report = compare_tracking_runs(
        milestone3_run_dir=args.milestone3_run_dir,
        milestone4_run_dir=args.milestone4_run_dir,
        output_dir=args.output_dir,
    )
    print(f"Wrote comparison report: {report['comparison_markdown']}")
    return 0
