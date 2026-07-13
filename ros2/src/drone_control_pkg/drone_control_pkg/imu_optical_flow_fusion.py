from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from drone_control_pkg.optical_flow_dead_reckon import OpticalFlowSample
from drone_control_pkg.optical_flow_dead_reckon import (
    _integrate_gyro_buffer,
    _rotate_xy,
)


@dataclass(frozen=True)
class ImuSample:
    """IMU observation with backward-compatible optional event metadata."""

    stamp_s: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float
    source_stamp_s: Optional[float] = None
    receive_stamp_s: Optional[float] = None
    frame_id: str = ""
    orientation_covariance: Optional[tuple[float, ...]] = None
    angular_velocity_covariance: Optional[tuple[float, ...]] = None
    linear_acceleration_covariance: Optional[tuple[float, ...]] = None


@dataclass(frozen=True)
class FusionPose:
    x_m: float
    y_m: float
    z_m: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    yaw_rad: float
    frame_id: str


@dataclass(frozen=True)
class FusionUpdate:
    pose: FusionPose
    valid: bool
    update_type: str
    reject_reason: str
    dt_s: float
    accel_map_x_mps2: float
    accel_map_y_mps2: float
    accel_map_z_mps2: float
    body_dx_m: float
    body_dy_m: float
    map_dx_m: float
    map_dy_m: float
    flow_vx_mps: float
    flow_vy_mps: float
    covariance_diagonal: tuple[float, ...] = ()
    health_state: str = ""
    sensor_age_s: float = math.nan
    measurement_variance_x: float = math.nan
    measurement_variance_y: float = math.nan
    measurement_variance_z: float = math.nan
    innovation_x: float = 0.0
    innovation_y: float = 0.0
    innovation_z: float = 0.0
    nis: float = math.nan
    gyro_source: str = ""


class ImuOpticalFlowFusion:
    """Nine-state error-aware IMU/optical-flow/range estimator.

    State ordering is ``[px, py, pz, vx, vy, vz, bax, bay, baz]`` in the
    map frame.  IMU acceleration propagates state and covariance; optical flow
    updates horizontal velocity and range updates vertical position.  All gains
    are therefore covariance- and measurement-health-dependent rather than fixed
    complementary-filter blends.

    The legacy weight arguments remain accepted.  They act only as measurement
    confidence multipliers and no longer directly blend state values.
    """

    STATE_SIZE = 9

    def __init__(
        self,
        *,
        frame_id: str = "map",
        quality_min: int = 10,
        range_min_m: float = 0.2,
        range_max_m: float = 5.0,
        gyro_compensation_gain: float = 1.0,
        flow_scale_x: float = 1.0,
        flow_scale_y: float = 1.0,
        flow_position_weight: float = 0.75,
        flow_velocity_weight: float = 0.50,
        range_z_weight: float = 0.80,
        gravity_mps2: float = 9.80665,
        subtract_gravity: bool = True,
        accel_deadband_mps2: float = 0.05,
        max_accel_mps2: float = 6.0,
        max_velocity_mps: float = 4.0,
        max_imu_dt_s: float = 0.10,
        velocity_decay_per_s: float = 0.04,
        accel_noise_mps2: float = 0.8,
        accel_bias_rw_mps3: float = 0.03,
        flow_velocity_noise_mps: float = 0.20,
        range_noise_m: float = 0.08,
        accel_lpf_tau_s: float = 0.08,
        innovation_gate_nis: float = 9.210,
        range_innovation_gate_nis: float = 6.635,
        max_sensor_age_s: float = 0.25,
        reorder_tolerance_s: float = 1e-6,
        max_tilt_rad: float = math.radians(45.0),
        max_flow_gap_s: float = 0.50,
        flow_gap_noise_scale: float = 0.25,
        flow_quality_noise_scale: float = 3.0,
        flow_range_noise_scale: float = 1.0,
        gyro_fallback_noise_scale: float = 2.0,
        gyro_coverage_tolerance_s: float = 0.005,
        gyro_buffer_duration_s: float = 1.0,
        flow_sensor_yaw_rad: float = 0.0,
        initial_position_std_m: float = 0.25,
        initial_velocity_std_mps: float = 0.50,
        initial_accel_bias_std_mps2: float = 0.30,
    ) -> None:
        self.frame_id = frame_id or "map"
        self.quality_min = int(quality_min)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self.gyro_compensation_gain = float(gyro_compensation_gain)
        self.flow_scale_x = float(flow_scale_x)
        self.flow_scale_y = float(flow_scale_y)
        self.flow_position_weight = _clamp01(flow_position_weight)
        self.flow_velocity_weight = _clamp01(flow_velocity_weight)
        self.range_z_weight = _clamp01(range_z_weight)
        self.gravity_mps2 = float(gravity_mps2)
        self.subtract_gravity = bool(subtract_gravity)
        self.accel_deadband_mps2 = max(float(accel_deadband_mps2), 0.0)
        self.max_accel_mps2 = max(float(max_accel_mps2), 0.0)
        self.max_velocity_mps = max(float(max_velocity_mps), 0.0)
        self.max_imu_dt_s = max(float(max_imu_dt_s), 0.001)
        self.velocity_decay_per_s = max(float(velocity_decay_per_s), 0.0)

        self.accel_noise_mps2 = max(float(accel_noise_mps2), 1e-6)
        self.accel_bias_rw_mps3 = max(float(accel_bias_rw_mps3), 0.0)
        self.flow_velocity_noise_mps = max(
            float(flow_velocity_noise_mps), 1e-6
        )
        self.range_noise_m = max(float(range_noise_m), 1e-6)
        self.accel_lpf_tau_s = max(float(accel_lpf_tau_s), 0.0)
        self.innovation_gate_nis = max(float(innovation_gate_nis), 0.0)
        self.range_innovation_gate_nis = max(
            float(range_innovation_gate_nis), 0.0
        )
        self.max_sensor_age_s = max(float(max_sensor_age_s), 0.0)
        self.reorder_tolerance_s = max(float(reorder_tolerance_s), 0.0)
        self.max_tilt_rad = max(float(max_tilt_rad), 0.0)
        self.max_flow_gap_s = max(float(max_flow_gap_s), 0.0)
        self.flow_gap_noise_scale = max(float(flow_gap_noise_scale), 0.0)
        self.flow_quality_noise_scale = max(
            float(flow_quality_noise_scale), 0.0
        )
        self.flow_range_noise_scale = max(
            float(flow_range_noise_scale), 0.0
        )
        self.gyro_fallback_noise_scale = max(
            float(gyro_fallback_noise_scale), 1.0
        )
        self.gyro_coverage_tolerance_s = max(
            float(gyro_coverage_tolerance_s), 0.0
        )
        self.gyro_buffer_duration_s = max(float(gyro_buffer_duration_s), 0.05)
        self.flow_sensor_yaw_rad = float(flow_sensor_yaw_rad)
        self.initial_position_std_m = max(float(initial_position_std_m), 1e-6)
        self.initial_velocity_std_mps = max(
            float(initial_velocity_std_mps), 1e-6
        )
        self.initial_accel_bias_std_mps2 = max(
            float(initial_accel_bias_std_mps2), 1e-6
        )

        self.pose: Optional[FusionPose] = None
        self.origin_pose: Optional[FusionPose] = None
        self.origin_range_m: Optional[float] = None
        self.last_imu_stamp_s: Optional[float] = None
        self.flow_anchor_pose: Optional[FusionPose] = None

        self._state: Optional[np.ndarray] = None
        self._covariance: Optional[np.ndarray] = None
        self._filtered_accel_map: Optional[np.ndarray] = None
        self._latest_tilt_rad = 0.0
        self._gyro_buffer: list[tuple[float, float, float, float]] = []
        self._last_flow_stamp_s: Optional[float] = None
        self._last_accepted_flow_stamp_s: Optional[float] = None
        self._has_accepted_flow = False
        self._last_flow_sequence: Optional[int] = None
        self._frozen = False

    @property
    def state(self) -> Optional[np.ndarray]:
        """Return a copy of the current nine-element state vector."""

        return None if self._state is None else self._state.copy()

    @property
    def covariance(self) -> Optional[np.ndarray]:
        """Return a copy of the current 9x9 covariance matrix."""

        return None if self._covariance is None else self._covariance.copy()

    @property
    def frozen(self) -> bool:
        return self._frozen

    def reset(
        self,
        *,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        range_m: Optional[float] = None,
        frame_id: str = "",
    ) -> FusionPose:
        resolved_frame = frame_id or self.frame_id or "map"
        self.frame_id = resolved_frame
        self._state = np.zeros(self.STATE_SIZE, dtype=float)
        self._state[0:3] = (float(x_m), float(y_m), float(z_m))
        initial_variances = np.square(
            [
                self.initial_position_std_m,
                self.initial_position_std_m,
                self.initial_position_std_m,
                self.initial_velocity_std_mps,
                self.initial_velocity_std_mps,
                self.initial_velocity_std_mps,
                self.initial_accel_bias_std_mps2,
                self.initial_accel_bias_std_mps2,
                self.initial_accel_bias_std_mps2,
            ]
        )
        self._covariance = np.diag(initial_variances)
        self.pose = FusionPose(
            x_m=float(x_m),
            y_m=float(y_m),
            z_m=float(z_m),
            vx_mps=0.0,
            vy_mps=0.0,
            vz_mps=0.0,
            yaw_rad=float(yaw_rad),
            frame_id=resolved_frame,
        )
        self.origin_pose = self.pose
        self.flow_anchor_pose = self.pose
        self.origin_range_m = (
            float(range_m)
            if range_m is not None and math.isfinite(float(range_m))
            else None
        )
        self.last_imu_stamp_s = None
        self._filtered_accel_map = None
        self._latest_tilt_rad = 0.0
        self._gyro_buffer.clear()
        self._last_flow_stamp_s = None
        self._last_accepted_flow_stamp_s = None
        self._has_accepted_flow = False
        self._last_flow_sequence = None
        self._frozen = False
        return self.pose

    def set_frozen(self, frozen: bool = True) -> Optional[FusionPose]:
        """Enable/disable landing hold; entering hold zeros all velocity."""

        self._frozen = bool(frozen)
        if self._frozen and self._state is not None:
            self._state[3:6] = 0.0
            if self._covariance is not None:
                self._covariance[3:6, :] = 0.0
                self._covariance[:, 3:6] = 0.0
                self._covariance[3:6, 3:6] = np.eye(3) * 1e-6
            self._sync_pose()
        return self.pose

    def freeze(self) -> Optional[FusionPose]:
        return self.set_frozen(True)

    def unfreeze(self) -> Optional[FusionPose]:
        return self.set_frozen(False)

    def update_imu(self, sample: ImuSample) -> FusionUpdate:
        if self.pose is None or self._state is None or self._covariance is None:
            return self._invalid_update("imu", "not_initialized")

        quat = _normalized_quaternion(
            sample.orientation_x,
            sample.orientation_y,
            sample.orientation_z,
            sample.orientation_w,
        )
        if quat is None:
            return self._invalid_update("imu", "orientation_invalid")

        source_stamp = _imu_source_stamp(sample)
        if source_stamp is None:
            return self._invalid_update("imu", "stamp_not_finite")
        sensor_age = _sensor_age(source_stamp, sample.receive_stamp_s)
        if self._is_stale(sensor_age):
            return self._invalid_update(
                "imu", "sensor_stale", sensor_age_s=sensor_age
            )

        if self.last_imu_stamp_s is not None and source_stamp <= self.last_imu_stamp_s:
            reason = (
                "imu_out_of_order"
                if source_stamp < self.last_imu_stamp_s - self.reorder_tolerance_s
                else "imu_duplicate"
            )
            return self._invalid_update(
                "imu", reason, sensor_age_s=sensor_age
            )

        yaw_rad = _yaw_from_quaternion(*quat)
        self._latest_tilt_rad = _tilt_from_quaternion(quat)
        gyro = (
            float(sample.angular_velocity_x),
            float(sample.angular_velocity_y),
            float(sample.angular_velocity_z),
        )
        if all(math.isfinite(value) for value in gyro):
            self._append_gyro(source_stamp, gyro)

        dt_s = 0.0
        if self.last_imu_stamp_s is not None:
            dt_s = source_stamp - self.last_imu_stamp_s
        self.last_imu_stamp_s = source_stamp
        self.pose = self._replace_pose(yaw_rad=yaw_rad)

        accel_body = np.asarray(
            (
                float(sample.linear_acceleration_x),
                float(sample.linear_acceleration_y),
                float(sample.linear_acceleration_z),
            ),
            dtype=float,
        )
        if not np.all(np.isfinite(accel_body)):
            return self._invalid_update(
                "imu", "accel_not_finite", dt_s=dt_s, sensor_age_s=sensor_age
            )

        accel_map = np.asarray(_rotate_vector(quat, tuple(accel_body)), dtype=float)
        if self.subtract_gravity:
            accel_map[2] -= self.gravity_mps2
        accel_map[np.abs(accel_map) < self.accel_deadband_mps2] = 0.0
        accel_map = np.asarray(
            _clamp_vector(tuple(accel_map), self.max_accel_mps2), dtype=float
        )

        if self._filtered_accel_map is None:
            self._filtered_accel_map = accel_map.copy()
        elif dt_s > 0.0:
            alpha = (
                1.0
                if self.accel_lpf_tau_s <= 0.0
                else dt_s / (self.accel_lpf_tau_s + dt_s)
            )
            self._filtered_accel_map += alpha * (
                accel_map - self._filtered_accel_map
            )

        if dt_s <= 0.0:
            return self._valid_update(
                update_type="imu",
                dt_s=0.0,
                accel_map=tuple(self._filtered_accel_map),
                sensor_age_s=sensor_age,
            )
        if dt_s > self.max_imu_dt_s:
            return self._invalid_update(
                "imu",
                "imu_gap_exceeded",
                dt_s=dt_s,
                sensor_age_s=sensor_age,
            )

        if not self._frozen:
            accel_covariance = _map_accel_covariance(
                sample.linear_acceleration_covariance,
                quat,
                self.accel_noise_mps2,
            )
            self._propagate(dt_s, self._filtered_accel_map, accel_covariance)
        self._sync_pose(yaw_rad=yaw_rad)
        return self._valid_update(
            update_type="imu",
            dt_s=dt_s,
            accel_map=tuple(self._filtered_accel_map),
            sensor_age_s=sensor_age,
        )

    def update_flow(self, sample: OpticalFlowSample) -> FusionUpdate:
        if self.pose is None or self._state is None or self._covariance is None:
            return self._invalid_update("flow", "not_initialized")

        source_stamp = _finite_optional(sample.source_stamp_s)
        sensor_age = _sensor_age(source_stamp, sample.receive_stamp_s)
        if self._is_stale(sensor_age):
            return self._invalid_update(
                "flow", "sensor_stale", sensor_age_s=sensor_age
            )

        previous_flow_stamp = self._last_flow_stamp_s
        timestamp_reason = self._flow_timestamp_reject_reason(
            sample, source_stamp
        )
        if timestamp_reason:
            return self._invalid_update(
                "flow", timestamp_reason, sensor_age_s=sensor_age
            )
        if (
            source_stamp is not None
            and self.last_imu_stamp_s is not None
            and source_stamp
            < self.last_imu_stamp_s - self.reorder_tolerance_s
        ):
            return self._invalid_update(
                "flow", "flow_state_skew", sensor_age_s=sensor_age
            )

        gap_s: Optional[float] = None
        if source_stamp is not None:
            if previous_flow_stamp is not None:
                gap_s = source_stamp - previous_flow_stamp
            self._last_flow_stamp_s = source_stamp
        if sample.sequence is not None:
            self._last_flow_sequence = int(sample.sequence)

        if (
            gap_s is not None
            and self.max_flow_gap_s > 0.0
            and gap_s > self.max_flow_gap_s
        ):
            return self._invalid_update(
                "flow",
                "flow_gap_exceeded",
                dt_s=gap_s,
                sensor_age_s=sensor_age,
            )

        # Range is an independent scalar measurement and remains useful even if
        # horizontal flow is later rejected.
        range_result = self._update_range(sample)

        reject_reason = self._flow_reject_reason(sample)
        if reject_reason:
            self._sync_pose()
            return self._invalid_update(
                "flow",
                reject_reason,
                dt_s=gap_s or 0.0,
                sensor_age_s=sensor_age,
                measurement_variance_z=range_result[2],
                innovation_z=range_result[1],
            )
        if self.max_tilt_rad > 0.0 and self._latest_tilt_rad > self.max_tilt_rad:
            self._sync_pose()
            return self._invalid_update(
                "flow",
                "tilt_above_max",
                dt_s=gap_s or 0.0,
                sensor_age_s=sensor_age,
                measurement_variance_z=range_result[2],
                innovation_z=range_result[1],
            )

        gyro_integral, gyro_source = self._resolve_flow_gyro(
            sample, source_stamp
        )
        if gyro_integral is None:
            self._sync_pose()
            return self._invalid_update(
                "flow",
                "gyro_integral_unavailable",
                dt_s=gap_s or 0.0,
                sensor_age_s=sensor_age,
                measurement_variance_z=range_result[2],
                innovation_z=range_result[1],
            )

        exposure_s = float(sample.integration_time_s)
        flow_x = float(sample.integrated_x) - (
            self.gyro_compensation_gain * gyro_integral[0]
        )
        flow_y = float(sample.integrated_y) - (
            self.gyro_compensation_gain * gyro_integral[1]
        )
        sensor_velocity = (
            self.flow_scale_x * float(sample.distance_m) * flow_y / exposure_s,
            self.flow_scale_y * float(sample.distance_m) * -flow_x / exposure_s,
        )
        body_velocity = _rotate_xy(
            sensor_velocity[0],
            sensor_velocity[1],
            self.flow_sensor_yaw_rad,
        )
        map_velocity = _rotate_xy(
            body_velocity[0], body_velocity[1], self.pose.yaw_rad
        )
        body_delta = (
            body_velocity[0] * exposure_s,
            body_velocity[1] * exposure_s,
        )
        map_delta = (
            map_velocity[0] * exposure_s,
            map_velocity[1] * exposure_s,
        )

        if (
            self.max_velocity_mps > 0.0
            and math.hypot(*map_velocity) > self.max_velocity_mps
        ):
            self._sync_pose()
            return self._flow_update_result(
                valid=False,
                reject_reason="flow_velocity_above_max",
                dt_s=gap_s or exposure_s,
                sensor_age_s=sensor_age,
                body_delta=body_delta,
                map_delta=map_delta,
                map_velocity=map_velocity,
                gyro_source=gyro_source,
                measurement_variance_z=range_result[2],
                innovation_z=range_result[1],
            )

        # Preserve sensible behavior for old, unstamped, flow-only callers.  In
        # timestamped fusion, position is propagated exclusively by IMU state.
        if source_stamp is None and self.last_imu_stamp_s is None:
            self._state[0] += map_delta[0]
            self._state[1] += map_delta[1]

        flow_covariance = self._dynamic_flow_covariance(
            sample,
            exposure_s=exposure_s,
            gap_s=gap_s,
            sensor_age_s=sensor_age,
            gyro_source=gyro_source,
        )
        h = np.zeros((2, self.STATE_SIZE), dtype=float)
        h[0, 3] = 1.0
        h[1, 4] = 1.0
        accepted, innovation, nis = self._measurement_update(
            np.asarray(map_velocity),
            h,
            flow_covariance,
            self.innovation_gate_nis,
        )
        self._limit_velocity()
        self._sync_pose()
        self.flow_anchor_pose = self.pose
        if accepted:
            self._last_accepted_flow_stamp_s = source_stamp
            self._has_accepted_flow = True
            return self._flow_update_result(
                valid=True,
                reject_reason="",
                dt_s=gap_s if gap_s is not None else exposure_s,
                sensor_age_s=sensor_age,
                body_delta=body_delta,
                map_delta=map_delta,
                map_velocity=map_velocity,
                gyro_source=gyro_source,
                measurement_variance_x=float(flow_covariance[0, 0]),
                measurement_variance_y=float(flow_covariance[1, 1]),
                measurement_variance_z=range_result[2],
                innovation=(float(innovation[0]), float(innovation[1])),
                innovation_z=range_result[1],
                nis=nis,
            )
        return self._flow_update_result(
            valid=False,
            reject_reason="innovation_gate",
            dt_s=gap_s if gap_s is not None else exposure_s,
            sensor_age_s=sensor_age,
            body_delta=body_delta,
            map_delta=map_delta,
            map_velocity=map_velocity,
            gyro_source=gyro_source,
            measurement_variance_x=float(flow_covariance[0, 0]),
            measurement_variance_y=float(flow_covariance[1, 1]),
            measurement_variance_z=range_result[2],
            innovation=(float(innovation[0]), float(innovation[1])),
            innovation_z=range_result[1],
            nis=nis,
        )

    def _propagate(
        self,
        dt_s: float,
        measured_accel_map: np.ndarray,
        accel_covariance: np.ndarray,
    ) -> None:
        assert self._state is not None and self._covariance is not None
        bias = self._state[6:9]
        acceleration = measured_accel_map - bias
        old_velocity = self._state[3:6].copy()
        decay = math.exp(-self.velocity_decay_per_s * dt_s)
        self._state[0:3] += (old_velocity * dt_s) + (
            0.5 * acceleration * dt_s * dt_s
        )
        self._state[3:6] = (old_velocity * decay) + acceleration * dt_s
        self._limit_velocity()

        transition = np.eye(self.STATE_SIZE)
        transition[0:3, 3:6] = np.eye(3) * dt_s
        transition[0:3, 6:9] = np.eye(3) * (-0.5 * dt_s * dt_s)
        transition[3:6, 3:6] = np.eye(3) * decay
        transition[3:6, 6:9] = np.eye(3) * -dt_s

        accel_map = np.zeros((self.STATE_SIZE, 3), dtype=float)
        accel_map[0:3, :] = np.eye(3) * (0.5 * dt_s * dt_s)
        accel_map[3:6, :] = np.eye(3) * dt_s
        process_noise = accel_map @ accel_covariance @ accel_map.T
        process_noise[6:9, 6:9] += (
            np.eye(3) * self.accel_bias_rw_mps3**2 * dt_s
        )
        self._covariance = (
            transition @ self._covariance @ transition.T + process_noise
        )
        self._stabilize_covariance()

    def _update_range(
        self, sample: OpticalFlowSample
    ) -> tuple[bool, float, float, float]:
        if (
            self._state is None
            or self.origin_pose is None
            or self.origin_range_m is None
        ):
            return False, 0.0, math.nan, math.nan
        distance = float(sample.distance_m)
        if (
            not math.isfinite(distance)
            or distance < self.range_min_m
            or distance > self.range_max_m
            or self.range_z_weight <= 0.0
        ):
            return False, 0.0, math.nan, math.nan

        vertical_distance = distance * math.cos(self._latest_tilt_rad)
        measured_z = self.origin_pose.z_m + (
            vertical_distance - self.origin_range_m
        )
        supplied_variance = _positive_optional(sample.range_variance_m2)
        variance = (
            supplied_variance
            if supplied_variance is not None
            else self.range_noise_m**2
        )
        variance *= _legacy_weight_variance_scale(self.range_z_weight)
        h = np.zeros((1, self.STATE_SIZE), dtype=float)
        h[0, 2] = 1.0
        accepted, innovation, nis = self._measurement_update(
            np.asarray((measured_z,)),
            h,
            np.asarray(((variance,),)),
            self.range_innovation_gate_nis,
        )
        return accepted, float(innovation[0]), float(variance), nis

    def _measurement_update(
        self,
        measurement: np.ndarray,
        h: np.ndarray,
        measurement_covariance: np.ndarray,
        gate_nis: float,
    ) -> tuple[bool, np.ndarray, float]:
        assert self._state is not None and self._covariance is not None
        innovation = measurement - h @ self._state
        residual_covariance = (
            h @ self._covariance @ h.T + measurement_covariance
        )
        try:
            solved = np.linalg.solve(residual_covariance, innovation)
            nis = float(innovation.T @ solved)
        except np.linalg.LinAlgError:
            return False, innovation, math.inf
        if not math.isfinite(nis) or (gate_nis > 0.0 and nis > gate_nis):
            return False, innovation, nis

        gain = np.linalg.solve(
            residual_covariance,
            h @ self._covariance,
        ).T
        self._state += gain @ innovation
        identity = np.eye(self.STATE_SIZE)
        residual_transition = identity - gain @ h
        self._covariance = (
            residual_transition
            @ self._covariance
            @ residual_transition.T
            + gain @ measurement_covariance @ gain.T
        )
        self._stabilize_covariance()
        return True, innovation, nis

    def _dynamic_flow_covariance(
        self,
        sample: OpticalFlowSample,
        *,
        exposure_s: float,
        gap_s: Optional[float],
        sensor_age_s: float,
        gyro_source: str,
    ) -> np.ndarray:
        quality_fraction = min(max(float(sample.quality) / 255.0, 0.0), 1.0)
        quality_factor = 1.0 + self.flow_quality_noise_scale * (
            1.0 - quality_fraction
        )
        range_span = max(self.range_max_m - self.range_min_m, 1e-6)
        range_fraction = min(
            max((float(sample.distance_m) - self.range_min_m) / range_span, 0.0),
            1.0,
        )
        range_factor = 1.0 + self.flow_range_noise_scale * range_fraction
        gap_factor = 1.0
        if gap_s is not None and exposure_s > 0.0:
            coverage_ratio = max(gap_s / exposure_s, 1.0)
            gap_factor += self.flow_gap_noise_scale * (coverage_ratio - 1.0)
        age_factor = 1.0
        if math.isfinite(sensor_age_s) and self.max_sensor_age_s > 0.0:
            age_factor += sensor_age_s / self.max_sensor_age_s
        gyro_factor = (
            self.gyro_fallback_noise_scale
            if gyro_source == "imu_buffer"
            else 1.0
        )
        sigma = (
            self.flow_velocity_noise_mps
            * quality_factor
            * range_factor
            * gap_factor
            * age_factor
            * gyro_factor
        )
        covariance = np.eye(2) * sigma**2

        supplied = _flow_covariance(sample.flow_covariance)
        if supplied is not None:
            jacobian = np.asarray(
                (
                    (0.0, self.flow_scale_x * sample.distance_m / exposure_s),
                    (-self.flow_scale_y * sample.distance_m / exposure_s, 0.0),
                )
            )
            total_yaw = self.flow_sensor_yaw_rad + self.pose.yaw_rad  # type: ignore[union-attr]
            rotation = np.asarray(
                (
                    (math.cos(total_yaw), -math.sin(total_yaw)),
                    (math.sin(total_yaw), math.cos(total_yaw)),
                )
            )
            transform = rotation @ jacobian
            covariance += transform @ supplied @ transform.T
        covariance *= _legacy_weight_variance_scale(self.flow_velocity_weight)
        return covariance

    def _resolve_flow_gyro(
        self,
        sample: OpticalFlowSample,
        source_stamp: Optional[float],
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        if self.gyro_compensation_gain == 0.0:
            return (0.0, 0.0, 0.0), "disabled"
        sensor_gyro = (
            float(sample.integrated_xgyro),
            float(sample.integrated_ygyro),
            float(sample.integrated_zgyro),
        )
        if all(math.isfinite(value) for value in sensor_gyro[:2]):
            z_value = sensor_gyro[2] if math.isfinite(sensor_gyro[2]) else 0.0
            return (sensor_gyro[0], sensor_gyro[1], z_value), "flow_sensor"
        if source_stamp is None:
            return None, ""
        exposure_s = float(sample.integration_time_s)
        integrated = _integrate_gyro_buffer(
            self._gyro_buffer,
            source_stamp - exposure_s,
            source_stamp,
            self.gyro_coverage_tolerance_s,
        )
        if integrated is None:
            return None, ""
        return integrated, "imu_buffer"

    def _flow_reject_reason(self, sample: OpticalFlowSample) -> str:
        if int(sample.quality) < self.quality_min:
            return "quality_below_min"
        if not math.isfinite(float(sample.distance_m)):
            return "range_not_finite"
        if float(sample.distance_m) < self.range_min_m:
            return "range_below_min"
        if float(sample.distance_m) > self.range_max_m:
            return "range_above_max"
        if not math.isfinite(float(sample.integration_time_s)):
            return "integration_time_not_finite"
        if float(sample.integration_time_s) <= 0.0:
            return "integration_time_invalid"
        if not math.isfinite(float(sample.integrated_x)):
            return "flow_x_not_finite"
        if not math.isfinite(float(sample.integrated_y)):
            return "flow_y_not_finite"
        return ""

    def _flow_timestamp_reject_reason(
        self,
        sample: OpticalFlowSample,
        source_stamp: Optional[float],
    ) -> str:
        if sample.source_stamp_s is not None and source_stamp is None:
            return "source_stamp_not_finite"
        if source_stamp is not None and self._last_flow_stamp_s is not None:
            if source_stamp <= self._last_flow_stamp_s:
                return (
                    "flow_out_of_order"
                    if source_stamp
                    < self._last_flow_stamp_s - self.reorder_tolerance_s
                    else "flow_duplicate"
                )
        if sample.sequence is not None and self._last_flow_sequence is not None:
            if int(sample.sequence) <= self._last_flow_sequence:
                return "flow_sequence_duplicate_or_out_of_order"
        return ""

    def _append_gyro(
        self, stamp_s: float, gyro: tuple[float, float, float]
    ) -> None:
        self._gyro_buffer.append((stamp_s, gyro[0], gyro[1], gyro[2]))
        cutoff = stamp_s - self.gyro_buffer_duration_s
        while len(self._gyro_buffer) > 2 and self._gyro_buffer[1][0] < cutoff:
            self._gyro_buffer.pop(0)

    def _is_stale(self, sensor_age_s: float) -> bool:
        return (
            math.isfinite(sensor_age_s)
            and self.max_sensor_age_s > 0.0
            and sensor_age_s > self.max_sensor_age_s
        )

    def _limit_velocity(self) -> None:
        if self._state is None:
            return
        self._state[3:6] = _clamp_vector(
            tuple(self._state[3:6]), self.max_velocity_mps
        )

    def _stabilize_covariance(self) -> None:
        assert self._covariance is not None
        self._covariance = 0.5 * (
            self._covariance + self._covariance.T
        )
        diagonal = np.maximum(np.diag(self._covariance), 1e-12)
        np.fill_diagonal(self._covariance, diagonal)

    def _sync_pose(self, yaw_rad: Optional[float] = None) -> FusionPose:
        assert self._state is not None
        previous_yaw = self.pose.yaw_rad if self.pose is not None else 0.0
        self.pose = FusionPose(
            x_m=float(self._state[0]),
            y_m=float(self._state[1]),
            z_m=float(self._state[2]),
            vx_mps=float(self._state[3]),
            vy_mps=float(self._state[4]),
            vz_mps=float(self._state[5]),
            yaw_rad=previous_yaw if yaw_rad is None else float(yaw_rad),
            frame_id=self.frame_id,
        )
        return self.pose

    def _replace_pose(
        self,
        *,
        yaw_rad: Optional[float] = None,
    ) -> FusionPose:
        assert self.pose is not None
        self.pose = FusionPose(
            x_m=self.pose.x_m,
            y_m=self.pose.y_m,
            z_m=self.pose.z_m,
            vx_mps=self.pose.vx_mps,
            vy_mps=self.pose.vy_mps,
            vz_mps=self.pose.vz_mps,
            yaw_rad=self.pose.yaw_rad if yaw_rad is None else float(yaw_rad),
            frame_id=self.frame_id,
        )
        return self.pose

    def _health_state(self) -> str:
        if self.pose is None:
            return "not_initialized"
        if self._frozen:
            return "frozen"
        if not self._has_accepted_flow:
            return "imu_only"
        if self._last_accepted_flow_stamp_s is None:
            return "tracking"
        if self.last_imu_stamp_s is None or self.max_flow_gap_s <= 0.0:
            return "tracking"
        if self.last_imu_stamp_s - self._last_accepted_flow_stamp_s <= self.max_flow_gap_s:
            return "tracking"
        return "coasting"

    def _covariance_diagonal(self) -> tuple[float, ...]:
        if self._covariance is None:
            return ()
        return tuple(float(value) for value in np.diag(self._covariance))

    def _valid_update(
        self,
        *,
        update_type: str,
        dt_s: float,
        accel_map: tuple[float, float, float],
        sensor_age_s: float = math.nan,
    ) -> FusionUpdate:
        assert self.pose is not None
        return FusionUpdate(
            pose=self.pose,
            valid=True,
            update_type=update_type,
            reject_reason="",
            dt_s=dt_s,
            accel_map_x_mps2=accel_map[0],
            accel_map_y_mps2=accel_map[1],
            accel_map_z_mps2=accel_map[2],
            body_dx_m=0.0,
            body_dy_m=0.0,
            map_dx_m=0.0,
            map_dy_m=0.0,
            flow_vx_mps=0.0,
            flow_vy_mps=0.0,
            covariance_diagonal=self._covariance_diagonal(),
            health_state=self._health_state(),
            sensor_age_s=sensor_age_s,
        )

    def _flow_update_result(
        self,
        *,
        valid: bool,
        reject_reason: str,
        dt_s: float,
        sensor_age_s: float,
        body_delta: tuple[float, float],
        map_delta: tuple[float, float],
        map_velocity: tuple[float, float],
        gyro_source: str,
        measurement_variance_x: float = math.nan,
        measurement_variance_y: float = math.nan,
        measurement_variance_z: float = math.nan,
        innovation: tuple[float, float] = (0.0, 0.0),
        innovation_z: float = 0.0,
        nis: float = math.nan,
    ) -> FusionUpdate:
        assert self.pose is not None
        return FusionUpdate(
            pose=self.pose,
            valid=valid,
            update_type="flow",
            reject_reason=reject_reason,
            dt_s=dt_s,
            accel_map_x_mps2=0.0,
            accel_map_y_mps2=0.0,
            accel_map_z_mps2=0.0,
            body_dx_m=body_delta[0],
            body_dy_m=body_delta[1],
            map_dx_m=map_delta[0],
            map_dy_m=map_delta[1],
            flow_vx_mps=map_velocity[0],
            flow_vy_mps=map_velocity[1],
            covariance_diagonal=self._covariance_diagonal(),
            health_state=self._health_state(),
            sensor_age_s=sensor_age_s,
            measurement_variance_x=measurement_variance_x,
            measurement_variance_y=measurement_variance_y,
            measurement_variance_z=measurement_variance_z,
            innovation_x=innovation[0],
            innovation_y=innovation[1],
            innovation_z=innovation_z,
            nis=nis,
            gyro_source=gyro_source,
        )

    def _invalid_update(
        self,
        update_type: str,
        reject_reason: str,
        *,
        dt_s: float = 0.0,
        sensor_age_s: float = math.nan,
        measurement_variance_z: float = math.nan,
        innovation_z: float = 0.0,
    ) -> FusionUpdate:
        pose = self.pose or FusionPose(
            x_m=0.0,
            y_m=0.0,
            z_m=0.0,
            vx_mps=0.0,
            vy_mps=0.0,
            vz_mps=0.0,
            yaw_rad=0.0,
            frame_id=self.frame_id,
        )
        return FusionUpdate(
            pose=pose,
            valid=False,
            update_type=update_type,
            reject_reason=reject_reason,
            dt_s=dt_s,
            accel_map_x_mps2=0.0,
            accel_map_y_mps2=0.0,
            accel_map_z_mps2=0.0,
            body_dx_m=0.0,
            body_dy_m=0.0,
            map_dx_m=0.0,
            map_dy_m=0.0,
            flow_vx_mps=0.0,
            flow_vy_mps=0.0,
            covariance_diagonal=self._covariance_diagonal(),
            health_state=self._health_state(),
            sensor_age_s=sensor_age_s,
            measurement_variance_z=measurement_variance_z,
            innovation_z=innovation_z,
        )


def _normalized_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[tuple[float, float, float, float]]:
    values = (float(x), float(y), float(z), float(w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-9:
        return None
    return tuple(value / norm for value in values)


def _yaw_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> float:
    siny_cosp = 2.0 * ((w * z) + (x * y))
    cosy_cosp = 1.0 - (2.0 * ((y * y) + (z * z)))
    return math.atan2(siny_cosp, cosy_cosp)


def _tilt_from_quaternion(
    quat: tuple[float, float, float, float]
) -> float:
    x, y, _, w = quat
    body_z_dot_map_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(min(max(body_z_dot_map_z, -1.0), 1.0))


def _rotation_matrix(
    quat: tuple[float, float, float, float]
) -> np.ndarray:
    x, y, z, w = quat
    return np.asarray(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _rotate_vector(
    quat: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(_rotation_matrix(quat) @ np.asarray(vector, dtype=float))


def _clamp_vector(
    vector: tuple[float, float, float],
    max_norm: float,
) -> tuple[float, float, float]:
    if max_norm <= 0.0:
        return vector
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= max_norm or norm <= 1e-9:
        return vector
    scale = max_norm / norm
    return tuple(value * scale for value in vector)


def _clamp01(value: float) -> float:
    return min(max(float(value), 0.0), 1.0)


def _finite_optional(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _positive_optional(value: Optional[float]) -> Optional[float]:
    parsed = _finite_optional(value)
    if parsed is None or parsed <= 0.0:
        return None
    return parsed


def _imu_source_stamp(sample: ImuSample) -> Optional[float]:
    explicit = _finite_optional(sample.source_stamp_s)
    if explicit is not None:
        return explicit
    return _finite_optional(sample.stamp_s)


def _sensor_age(
    source_stamp_s: Optional[float], receive_stamp_s: Optional[float]
) -> float:
    receive = _finite_optional(receive_stamp_s)
    if source_stamp_s is None or receive is None:
        return math.nan
    return max(receive - source_stamp_s, 0.0)


def _legacy_weight_variance_scale(weight: float) -> float:
    if weight <= 0.0:
        return 1e12
    return max((1.0 - weight) / weight, 1e-9)


def _flow_covariance(
    values: Optional[Sequence[float]],
) -> Optional[np.ndarray]:
    if values is None or len(values) < 4:
        return None
    covariance = np.asarray(
        ((float(values[0]), float(values[1])), (float(values[2]), float(values[3]))),
        dtype=float,
    )
    if not np.all(np.isfinite(covariance)):
        return None
    covariance = 0.5 * (covariance + covariance.T)
    if np.any(np.diag(covariance) < 0.0):
        return None
    return covariance


def _map_accel_covariance(
    values: Optional[Sequence[float]],
    quat: tuple[float, float, float, float],
    fallback_sigma: float,
) -> np.ndarray:
    if values is None or len(values) < 9 or float(values[0]) < 0.0:
        return np.eye(3) * fallback_sigma**2
    covariance = np.asarray(values[:9], dtype=float).reshape((3, 3))
    if not np.all(np.isfinite(covariance)) or np.any(np.diag(covariance) < 0.0):
        return np.eye(3) * fallback_sigma**2
    covariance = 0.5 * (covariance + covariance.T)
    rotation = _rotation_matrix(quat)
    return rotation @ covariance @ rotation.T + np.eye(3) * 1e-12
