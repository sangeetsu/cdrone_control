from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np


TRACK_SOURCE_DETECTED = 0
TRACK_SOURCE_HELD = 1
TRACK_SOURCE_PREDICTED = 2


def finite_vector(values: np.ndarray, *, expected_size: int = 3) -> bool:
    array = np.asarray(values, dtype=np.float64)
    return array.shape == (expected_size,) and bool(np.all(np.isfinite(array)))


def normalized_quaternion_to_rotation_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-9:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    else:
        x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def world_to_body_position(
    world_position_m: np.ndarray,
    ownship_position_m: np.ndarray,
    world_from_body_rotation: np.ndarray,
) -> np.ndarray:
    centered = np.asarray(world_position_m, dtype=np.float64) - np.asarray(
        ownship_position_m, dtype=np.float64
    )
    return np.asarray(world_from_body_rotation, dtype=np.float64).T @ centered


@dataclass(frozen=True)
class WorldTrackObservation:
    track_id: int
    detector_track_id: int
    source: int
    position_m: np.ndarray
    velocity_mps: np.ndarray
    confidence: float
    bbox_area_px: float
    distance_m: float
    inbound: bool
    observed_at_s: float
    received_at_s: float
    last_observed_age_s: float
    prediction_horizon_s: float
    position_uncertainty_m: float
    velocity_uncertainty_mps: float


@dataclass
class TargetMemoryTrack:
    map_id: int
    detector_track_id: int
    position_m: np.ndarray
    velocity_mps: np.ndarray
    confidence: float
    bbox_area_px: float
    distance_m: float
    inbound: bool
    first_observed_s: float
    last_observed_s: float
    last_observation_received_s: float
    last_update_s: float
    source: int = TRACK_SOURCE_DETECTED
    position_uncertainty_m: float = 0.0
    velocity_uncertainty_mps: float = 0.0
    observed_count: int = 1
    history: list[tuple[float, np.ndarray]] = field(default_factory=list)

    def predict_to(
        self,
        now_s: float,
        *,
        position_process_noise_mps: float,
        velocity_process_noise_mps: float,
    ) -> None:
        dt_s = max(float(now_s) - float(self.last_update_s), 0.0)
        if dt_s <= 1e-6:
            return
        self.position_m = self.position_m + (self.velocity_mps * dt_s)
        self.position_uncertainty_m = max(
            0.0,
            float(self.position_uncertainty_m)
            + (max(position_process_noise_mps, 0.0) * dt_s),
        )
        self.velocity_uncertainty_mps = max(
            0.0,
            float(self.velocity_uncertainty_mps)
            + (max(velocity_process_noise_mps, 0.0) * dt_s),
        )
        self.last_update_s = float(now_s)

    def observed_age_s(self, now_s: float) -> float:
        return max(float(now_s) - float(self.last_observed_s), 0.0)

    def output_source(self, now_s: float, *, observation_fresh_s: float) -> int:
        if max(float(now_s) - float(self.last_observation_received_s), 0.0) <= max(
            float(observation_fresh_s), 0.0
        ):
            return int(self.source)
        if self.observed_age_s(now_s) > max(float(observation_fresh_s), 0.0):
            return TRACK_SOURCE_PREDICTED
        return int(self.source)

    def decayed_confidence(
        self,
        now_s: float,
        *,
        confidence_decay_per_s: float,
    ) -> float:
        age_s = self.observed_age_s(now_s)
        scale = max(0.0, 1.0 - (max(confidence_decay_per_s, 0.0) * age_s))
        return float(max(0.0, min(1.0, self.confidence * scale)))


class TargetMemory:
    def __init__(
        self,
        *,
        association_gate_m: float = 0.8,
        association_uncertainty_scale: float = 1.0,
        max_prediction_horizon_s: float = 3.0,
        prune_after_s: float = 5.0,
        confidence_decay_per_s: float = 0.25,
        observed_position_uncertainty_m: float = 0.15,
        observed_velocity_uncertainty_mps: float = 0.35,
        position_process_noise_mps: float = 0.35,
        velocity_process_noise_mps: float = 0.25,
        velocity_blend_alpha: float = 0.55,
        max_velocity_mps: float = 8.0,
        history_length: int = 30,
    ) -> None:
        self.association_gate_m = float(association_gate_m)
        self.association_uncertainty_scale = float(association_uncertainty_scale)
        self.max_prediction_horizon_s = float(max_prediction_horizon_s)
        self.prune_after_s = float(prune_after_s)
        self.confidence_decay_per_s = float(confidence_decay_per_s)
        self.observed_position_uncertainty_m = float(observed_position_uncertainty_m)
        self.observed_velocity_uncertainty_mps = float(observed_velocity_uncertainty_mps)
        self.position_process_noise_mps = float(position_process_noise_mps)
        self.velocity_process_noise_mps = float(velocity_process_noise_mps)
        self.velocity_blend_alpha = max(0.0, min(1.0, float(velocity_blend_alpha)))
        self.max_velocity_mps = float(max_velocity_mps)
        self.history_length = max(2, int(history_length))
        self.tracks: dict[int, TargetMemoryTrack] = {}
        self.next_map_id = 1

    def update(
        self,
        observations: list[WorldTrackObservation],
        *,
        now_s: float,
    ) -> None:
        self.predict_all(now_s)
        available_track_ids = set(self.tracks.keys())

        for observation in sorted(
            observations,
            key=lambda item: (-float(item.confidence), int(item.track_id)),
        ):
            if not finite_vector(observation.position_m) or not finite_vector(
                observation.velocity_mps
            ):
                continue
            map_id = self._find_match(observation, available_track_ids, now_s=now_s)
            if map_id is None:
                map_id = self._create_track(observation, now_s=now_s)
            else:
                self._update_track(map_id, observation, now_s=now_s)
            available_track_ids.discard(map_id)

        self.prune(now_s)

    def predict_all(self, now_s: float) -> None:
        for track in self.tracks.values():
            track.predict_to(
                now_s,
                position_process_noise_mps=self.position_process_noise_mps,
                velocity_process_noise_mps=self.velocity_process_noise_mps,
            )

    def snapshot(
        self,
        *,
        now_s: float,
        observation_fresh_s: float,
    ) -> list[TargetMemoryTrack]:
        self.predict_all(now_s)
        visible_tracks = []
        for track in self.tracks.values():
            if track.observed_age_s(now_s) > self.max_prediction_horizon_s:
                continue
            if (
                track.output_source(now_s, observation_fresh_s=observation_fresh_s)
                == TRACK_SOURCE_PREDICTED
            ):
                self._append_history(track, now_s=now_s)
            visible_tracks.append(track)
        visible_tracks.sort(key=lambda track: track.map_id)
        return visible_tracks

    def prune(self, now_s: float) -> None:
        prune_after_s = max(self.prune_after_s, self.max_prediction_horizon_s)
        stale_ids = [
            map_id
            for map_id, track in self.tracks.items()
            if track.observed_age_s(now_s) > prune_after_s
        ]
        for map_id in stale_ids:
            self.tracks.pop(map_id, None)

    def _find_match(
        self,
        observation: WorldTrackObservation,
        available_track_ids: set[int],
        *,
        now_s: float,
    ) -> int | None:
        detector_track_id = int(observation.detector_track_id)
        if detector_track_id >= 0:
            detector_matches = [
                map_id
                for map_id in available_track_ids
                if self.tracks[map_id].detector_track_id == detector_track_id
            ]
            if detector_matches:
                return min(detector_matches)

        best_map_id = None
        best_distance_m = float("inf")
        for map_id in available_track_ids:
            track = self.tracks[map_id]
            if track.observed_age_s(now_s) > self.max_prediction_horizon_s:
                continue
            distance_m = float(np.linalg.norm(track.position_m - observation.position_m))
            gate_m = max(
                self.association_gate_m,
                (
                    track.position_uncertainty_m
                    + max(observation.position_uncertainty_m, 0.0)
                )
                * max(self.association_uncertainty_scale, 0.0),
            )
            if distance_m <= gate_m and distance_m < best_distance_m:
                best_map_id = map_id
                best_distance_m = distance_m
        return best_map_id

    def _create_track(
        self,
        observation: WorldTrackObservation,
        *,
        now_s: float,
    ) -> int:
        map_id = self.next_map_id
        self.next_map_id += 1
        position_uncertainty_m = (
            observation.position_uncertainty_m
            if observation.position_uncertainty_m > 0.0
            else self.observed_position_uncertainty_m
        )
        velocity_uncertainty_mps = (
            observation.velocity_uncertainty_mps
            if observation.velocity_uncertainty_mps > 0.0
            else self.observed_velocity_uncertainty_mps
        )
        track = TargetMemoryTrack(
            map_id=map_id,
            detector_track_id=int(observation.detector_track_id),
            position_m=np.asarray(observation.position_m, dtype=np.float64).copy(),
            velocity_mps=self._bounded_velocity(observation.velocity_mps),
            confidence=float(max(0.0, min(1.0, observation.confidence))),
            bbox_area_px=float(observation.bbox_area_px),
            distance_m=float(observation.distance_m),
            inbound=bool(observation.inbound),
            first_observed_s=float(observation.observed_at_s),
            last_observed_s=float(observation.observed_at_s),
            last_observation_received_s=float(now_s),
            last_update_s=float(now_s),
            source=int(observation.source),
            position_uncertainty_m=float(position_uncertainty_m),
            velocity_uncertainty_mps=float(velocity_uncertainty_mps),
            history=[],
        )
        self._append_history(track, now_s=now_s)
        self.tracks[map_id] = track
        return map_id

    def _update_track(
        self,
        map_id: int,
        observation: WorldTrackObservation,
        *,
        now_s: float,
    ) -> None:
        track = self.tracks[map_id]
        previous_position_m = track.position_m.copy()
        previous_observed_s = track.last_observed_s
        observed_dt_s = max(float(observation.observed_at_s) - previous_observed_s, 0.0)
        observed_velocity_mps = self._bounded_velocity(observation.velocity_mps)
        if np.linalg.norm(observed_velocity_mps) <= 1e-6 and observed_dt_s > 1e-3:
            observed_velocity_mps = self._bounded_velocity(
                (observation.position_m - previous_position_m) / observed_dt_s
            )

        alpha = self.velocity_blend_alpha
        track.velocity_mps = (
            (alpha * observed_velocity_mps) + ((1.0 - alpha) * track.velocity_mps)
        )
        track.position_m = np.asarray(observation.position_m, dtype=np.float64).copy()
        track.detector_track_id = int(observation.detector_track_id)
        track.confidence = float(max(0.0, min(1.0, observation.confidence)))
        track.bbox_area_px = float(observation.bbox_area_px)
        track.distance_m = float(observation.distance_m)
        track.inbound = bool(observation.inbound)
        track.source = int(observation.source)
        track.last_observed_s = float(observation.observed_at_s)
        track.last_observation_received_s = float(now_s)
        track.last_update_s = float(now_s)
        track.observed_count += 1
        track.position_uncertainty_m = float(
            observation.position_uncertainty_m
            if observation.position_uncertainty_m > 0.0
            else self.observed_position_uncertainty_m
        )
        track.velocity_uncertainty_mps = float(
            observation.velocity_uncertainty_mps
            if observation.velocity_uncertainty_mps > 0.0
            else self.observed_velocity_uncertainty_mps
        )
        self._append_history(track, now_s=now_s)

    def _bounded_velocity(self, velocity_mps: np.ndarray) -> np.ndarray:
        velocity = np.asarray(velocity_mps, dtype=np.float64).copy()
        if not finite_vector(velocity):
            return np.zeros(3, dtype=np.float64)
        velocity_norm = float(np.linalg.norm(velocity))
        max_velocity_mps = max(self.max_velocity_mps, 0.0)
        if max_velocity_mps > 0.0 and velocity_norm > max_velocity_mps:
            velocity *= max_velocity_mps / max(velocity_norm, 1e-6)
        return velocity

    def _append_history(self, track: TargetMemoryTrack, *, now_s: float) -> None:
        track.history.append((float(now_s), track.position_m.copy()))
        if len(track.history) > self.history_length:
            del track.history[: len(track.history) - self.history_length]
