from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np

try:
    import cv2
except Exception:  # pragma: no cover - platform specific
    cv2 = None


@dataclass(frozen=True)
class BodyTrackState:
    track_id: int
    x_b_m: float
    y_b_m: float
    z_b_m: float
    vx_b_mps: float
    vy_b_mps: float
    vz_b_mps: float
    confidence: float
    bbox_area_px: float
    inbound: bool
    last_seen_s: float


def clamp_bbox_to_image(
    bbox: list[float] | tuple[float, float, float, float] | None,
    image_shape: tuple[int, ...],
) -> tuple[int, int, int, int] | None:
    if bbox is None or len(bbox) != 4 or len(image_shape) < 2:
        return None
    height_px = int(image_shape[0])
    width_px = int(image_shape[1])
    if height_px <= 1 or width_px <= 1:
        return None

    x1 = int(math.floor(min(float(bbox[0]), float(bbox[2]))))
    y1 = int(math.floor(min(float(bbox[1]), float(bbox[3]))))
    x2 = int(math.ceil(max(float(bbox[0]), float(bbox[2]))))
    y2 = int(math.ceil(max(float(bbox[1]), float(bbox[3]))))

    x1 = max(0, min(x1, width_px - 1))
    y1 = max(0, min(y1, height_px - 1))
    x2 = max(1, min(x2, width_px))
    y2 = max(1, min(y2, height_px))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def compute_lab_histogram_embedding(
    image: np.ndarray | None,
    bbox: list[float] | tuple[float, float, float, float] | None,
    *,
    bins_per_channel: int,
) -> np.ndarray | None:
    if cv2 is None or image is None:
        return None
    clamped_bbox = clamp_bbox_to_image(bbox, image.shape)
    if clamped_bbox is None:
        return None
    x1, y1, x2, y2 = clamped_bbox
    cutout = image[y1:y2, x1:x2]
    if cutout.size == 0:
        return None

    lab_cutout = cv2.cvtColor(cutout, cv2.COLOR_BGR2Lab)
    bins_per_channel = max(int(bins_per_channel), 4)
    histogram = cv2.calcHist(
        [lab_cutout],
        [0, 1],
        None,
        [bins_per_channel, bins_per_channel],
        [0, 256, 0, 256],
    )
    histogram = cv2.normalize(histogram, histogram).flatten()
    if histogram.size == 0:
        return None
    return histogram.astype(np.float32)


def histogram_correlation_distance(
    first_embedding: np.ndarray | None,
    second_embedding: np.ndarray | None,
) -> float:
    if cv2 is None or first_embedding is None or second_embedding is None:
        return 1.0
    correlation = float(
        cv2.compareHist(
            np.asarray(first_embedding, dtype=np.float32),
            np.asarray(second_embedding, dtype=np.float32),
            cv2.HISTCMP_CORREL,
        )
    )
    if not math.isfinite(correlation):
        return 1.0
    return max(0.0, min(2.0, 1.0 - correlation))


def latest_track_embedding(track: object) -> np.ndarray | None:
    last_detection = getattr(track, "last_detection", None)
    embedding = getattr(last_detection, "embedding", None)
    if embedding is not None:
        return np.asarray(embedding, dtype=np.float32)

    for detection in reversed(list(getattr(track, "past_detections", []) or [])):
        embedding = getattr(detection, "embedding", None)
        if embedding is not None:
            return np.asarray(embedding, dtype=np.float32)
    return None


def tracked_object_reid_distance(first_track: object, second_track: object) -> float:
    return histogram_correlation_distance(
        latest_track_embedding(first_track),
        latest_track_embedding(second_track),
    )


def extrapolate_body_track_state(
    track_state: BodyTrackState,
    *,
    now_s: float,
    max_extrapolation_m: float,
) -> BodyTrackState:
    dt_s = max(float(now_s) - float(track_state.last_seen_s), 0.0)
    delta_m = np.array(
        [
            track_state.vx_b_mps * dt_s,
            track_state.vy_b_mps * dt_s,
            track_state.vz_b_mps * dt_s,
        ],
        dtype=np.float32,
    )
    max_extrapolation_m = max(float(max_extrapolation_m), 0.0)
    if max_extrapolation_m > 0.0:
        delta_norm_m = float(np.linalg.norm(delta_m))
        if delta_norm_m > max_extrapolation_m:
            delta_m *= max_extrapolation_m / delta_norm_m

    return BodyTrackState(
        track_id=int(track_state.track_id),
        x_b_m=float(track_state.x_b_m + delta_m[0]),
        y_b_m=float(track_state.y_b_m + delta_m[1]),
        z_b_m=float(track_state.z_b_m + delta_m[2]),
        vx_b_mps=float(track_state.vx_b_mps),
        vy_b_mps=float(track_state.vy_b_mps),
        vz_b_mps=float(track_state.vz_b_mps),
        confidence=float(track_state.confidence),
        bbox_area_px=float(track_state.bbox_area_px),
        inbound=bool(track_state.inbound),
        last_seen_s=float(track_state.last_seen_s),
    )


def select_held_track_states(
    *,
    active_track_ids: set[int],
    fresh_track_ids: set[int],
    cached_tracks: Mapping[int, BodyTrackState],
    now_s: float,
    hold_window_s: float,
    max_extrapolation_m: float,
) -> dict[int, BodyTrackState]:
    held_tracks: dict[int, BodyTrackState] = {}
    hold_window_s = max(float(hold_window_s), 0.0)
    if hold_window_s <= 0.0:
        return held_tracks

    for track_id in sorted(int(track_id) for track_id in active_track_ids):
        if track_id in fresh_track_ids:
            continue
        cached_track = cached_tracks.get(track_id)
        if cached_track is None:
            continue
        if max(float(now_s) - float(cached_track.last_seen_s), 0.0) > hold_window_s:
            continue
        held_tracks[track_id] = extrapolate_body_track_state(
            cached_track,
            now_s=now_s,
            max_extrapolation_m=max_extrapolation_m,
        )
    return held_tracks
