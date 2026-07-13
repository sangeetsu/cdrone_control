from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence


@dataclass(frozen=True)
class OpticalFlowSample:
    """One integrated optical-flow observation.

    The first eight fields deliberately retain the original constructor contract.
    Timestamp and covariance metadata are optional so old bags and callers remain
    usable.  ``source_stamp_s`` is the end of the integration exposure.
    ``flow_covariance`` is a flattened 2x2 covariance for integrated_x/y.
    """

    integrated_x: float
    integrated_y: float
    integrated_xgyro: float
    integrated_ygyro: float
    integrated_zgyro: float
    quality: int
    distance_m: float
    integration_time_s: float
    source_stamp_s: Optional[float] = None
    receive_stamp_s: Optional[float] = None
    frame_id: str = ""
    sequence: Optional[int] = None
    flow_covariance: Optional[tuple[float, ...]] = None
    range_variance_m2: Optional[float] = None


@dataclass(frozen=True)
class OpticalFlowPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    frame_id: str


@dataclass(frozen=True)
class OpticalFlowUpdate:
    pose: OpticalFlowPose
    valid: bool
    reject_reason: str
    body_dx_m: float
    body_dy_m: float
    map_dx_m: float
    map_dy_m: float
    yaw_used_rad: float
    dt_s: float = 0.0
    body_vx_mps: float = 0.0
    body_vy_mps: float = 0.0
    map_vx_mps: float = 0.0
    map_vy_mps: float = 0.0
    gyro_source: str = ""


class OpticalFlowDeadReckoner:
    """Timestamp-aware optical-flow velocity integration.

    Integrated flow describes angular displacement during the sensor exposure.
    Dividing by that exposure yields a sampled velocity.  When source timestamps
    are present, consecutive velocity samples are integrated with a bounded
    trapezoid over the *message interval*.  Old, unstamped inputs retain the
    exposure-window integration used by the original implementation.
    """

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
        flow_sensor_yaw_rad: float = 0.0,
        max_flow_dt_s: float = 0.25,
        reorder_tolerance_s: float = 1e-6,
        gyro_coverage_tolerance_s: float = 0.005,
        gyro_buffer_duration_s: float = 1.0,
    ) -> None:
        self.frame_id = frame_id or "map"
        self.quality_min = int(quality_min)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self.gyro_compensation_gain = float(gyro_compensation_gain)
        self.flow_scale_x = float(flow_scale_x)
        self.flow_scale_y = float(flow_scale_y)
        self.flow_sensor_yaw_rad = float(flow_sensor_yaw_rad)
        self.max_flow_dt_s = max(float(max_flow_dt_s), 1e-6)
        self.reorder_tolerance_s = max(float(reorder_tolerance_s), 0.0)
        self.gyro_coverage_tolerance_s = max(
            float(gyro_coverage_tolerance_s), 0.0
        )
        self.gyro_buffer_duration_s = max(float(gyro_buffer_duration_s), 0.05)

        self.pose: Optional[OpticalFlowPose] = None
        self.origin_pose: Optional[OpticalFlowPose] = None
        self.origin_range_m: Optional[float] = None
        self.last_source_stamp_s: Optional[float] = None
        self._last_body_velocity: Optional[tuple[float, float]] = None
        self._last_map_velocity: Optional[tuple[float, float]] = None
        self._gyro_buffer: list[tuple[float, float, float, float]] = []

    def reset(
        self,
        *,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
        range_m: Optional[float] = None,
        frame_id: str = "",
    ) -> OpticalFlowPose:
        resolved_frame = frame_id or self.frame_id or "map"
        self.frame_id = resolved_frame
        self.pose = OpticalFlowPose(
            x_m=float(x_m),
            y_m=float(y_m),
            z_m=float(z_m),
            yaw_rad=float(yaw_rad),
            frame_id=resolved_frame,
        )
        self.origin_pose = self.pose
        self.origin_range_m = (
            float(range_m)
            if range_m is not None and math.isfinite(float(range_m))
            else None
        )
        self.last_source_stamp_s = None
        self._last_body_velocity = None
        self._last_map_velocity = None
        self._gyro_buffer.clear()
        return self.pose

    def update_imu_gyro(
        self,
        *,
        stamp_s: float,
        angular_velocity_x: float,
        angular_velocity_y: float,
        angular_velocity_z: float,
    ) -> bool:
        """Buffer a body-frame IMU rate for missing flow gyro integrals."""

        values = (
            float(stamp_s),
            float(angular_velocity_x),
            float(angular_velocity_y),
            float(angular_velocity_z),
        )
        if not all(math.isfinite(value) for value in values):
            return False
        if self._gyro_buffer and values[0] <= self._gyro_buffer[-1][0]:
            return False
        self._gyro_buffer.append(values)
        cutoff = values[0] - self.gyro_buffer_duration_s
        while len(self._gyro_buffer) > 2 and self._gyro_buffer[1][0] < cutoff:
            self._gyro_buffer.pop(0)
        return True

    def update(
        self,
        sample: OpticalFlowSample,
        *,
        yaw_rad: float,
    ) -> OpticalFlowUpdate:
        if self.pose is None:
            pose = OpticalFlowPose(0.0, 0.0, 0.0, yaw_rad, self.frame_id)
            return self._update_result(
                pose=pose,
                valid=False,
                reject_reason="not_initialized",
                yaw_rad=yaw_rad,
            )

        reject_reason = self._reject_reason(sample)
        if reject_reason:
            return self._rejected(sample, yaw_rad, reject_reason)

        source_stamp = _finite_optional(sample.source_stamp_s)
        if (
            source_stamp is not None
            and self.last_source_stamp_s is not None
            and source_stamp
            <= self.last_source_stamp_s + self.reorder_tolerance_s
        ):
            return self._rejected(sample, yaw_rad, "flow_out_of_order")

        gyro_integral, gyro_source = self._resolve_gyro_integral(
            sample, source_stamp
        )
        if gyro_integral is None:
            return self._rejected(
                sample, yaw_rad, "gyro_integral_unavailable"
            )

        exposure_s = float(sample.integration_time_s)
        flow_x = float(sample.integrated_x) - (
            self.gyro_compensation_gain * gyro_integral[0]
        )
        flow_y = float(sample.integrated_y) - (
            self.gyro_compensation_gain * gyro_integral[1]
        )

        sensor_vx = self.flow_scale_x * float(sample.distance_m) * (
            flow_y / exposure_s
        )
        sensor_vy = self.flow_scale_y * float(sample.distance_m) * (
            -flow_x / exposure_s
        )
        body_vx, body_vy = _rotate_xy(
            sensor_vx, sensor_vy, self.flow_sensor_yaw_rad
        )
        map_vx, map_vy = _rotate_xy(body_vx, body_vy, float(yaw_rad))

        if source_stamp is None:
            # Compatibility path for legacy bags without message timestamps.
            dt_s = exposure_s
            body_dx_m = body_vx * dt_s
            body_dy_m = body_vy * dt_s
            map_dx_m = map_vx * dt_s
            map_dy_m = map_vy * dt_s
        elif self.last_source_stamp_s is None:
            # A single timestamped velocity sample has no inter-message interval.
            dt_s = 0.0
            body_dx_m = body_dy_m = 0.0
            map_dx_m = map_dy_m = 0.0
        elif self._last_map_velocity is None or self._last_body_velocity is None:
            # Do not bridge an interval containing a rejected/missing sample.
            dt_s = 0.0
            body_dx_m = body_dy_m = 0.0
            map_dx_m = map_dy_m = 0.0
        else:
            dt_s = min(
                source_stamp - self.last_source_stamp_s,
                self.max_flow_dt_s,
            )
            previous_body = self._last_body_velocity or (body_vx, body_vy)
            previous_map = self._last_map_velocity or (map_vx, map_vy)
            body_dx_m = 0.5 * (previous_body[0] + body_vx) * dt_s
            body_dy_m = 0.5 * (previous_body[1] + body_vy) * dt_s
            map_dx_m = 0.5 * (previous_map[0] + map_vx) * dt_s
            map_dy_m = 0.5 * (previous_map[1] + map_vy) * dt_s

        self.pose = OpticalFlowPose(
            x_m=self.pose.x_m + map_dx_m,
            y_m=self.pose.y_m + map_dy_m,
            z_m=self._z_from_range(sample.distance_m),
            yaw_rad=float(yaw_rad),
            frame_id=self.frame_id,
        )
        if source_stamp is not None:
            self.last_source_stamp_s = source_stamp
            self._last_body_velocity = (body_vx, body_vy)
            self._last_map_velocity = (map_vx, map_vy)
        return self._update_result(
            pose=self.pose,
            valid=True,
            reject_reason="",
            yaw_rad=yaw_rad,
            body_delta=(body_dx_m, body_dy_m),
            map_delta=(map_dx_m, map_dy_m),
            dt_s=dt_s,
            body_velocity=(body_vx, body_vy),
            map_velocity=(map_vx, map_vy),
            gyro_source=gyro_source,
        )

    def _resolve_gyro_integral(
        self,
        sample: OpticalFlowSample,
        source_stamp: Optional[float],
    ) -> tuple[Optional[tuple[float, float, float]], str]:
        if self.gyro_compensation_gain == 0.0:
            return (0.0, 0.0, 0.0), "disabled"
        sensor_values = (
            float(sample.integrated_xgyro),
            float(sample.integrated_ygyro),
            float(sample.integrated_zgyro),
        )
        if all(math.isfinite(value) for value in sensor_values[:2]):
            z_value = sensor_values[2] if math.isfinite(sensor_values[2]) else 0.0
            return (sensor_values[0], sensor_values[1], z_value), "flow_sensor"
        exposure_s = float(sample.integration_time_s)
        if source_stamp is None:
            return None, ""
        integrated = _integrate_gyro_buffer(
            self._gyro_buffer,
            source_stamp - exposure_s,
            source_stamp,
            self.gyro_coverage_tolerance_s,
        )
        if integrated is None:
            return None, ""
        return integrated, "imu_buffer"

    def _z_from_range(self, range_m: float) -> float:
        if self.pose is None:
            return 0.0
        if self.origin_pose is None or self.origin_range_m is None:
            return self.pose.z_m
        if not math.isfinite(float(range_m)):
            return self.pose.z_m
        return self.origin_pose.z_m + (float(range_m) - self.origin_range_m)

    def _reject_reason(self, sample: OpticalFlowSample) -> str:
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

    def _rejected(
        self,
        sample: OpticalFlowSample,
        yaw_rad: float,
        reason: str,
    ) -> OpticalFlowUpdate:
        assert self.pose is not None
        source_stamp = _finite_optional(sample.source_stamp_s)
        if (
            source_stamp is not None
            and reason != "flow_out_of_order"
            and (
                self.last_source_stamp_s is None
                or source_stamp > self.last_source_stamp_s
            )
        ):
            self.last_source_stamp_s = source_stamp
            self._last_body_velocity = None
            self._last_map_velocity = None
        self.pose = OpticalFlowPose(
            x_m=self.pose.x_m,
            y_m=self.pose.y_m,
            z_m=self._z_from_range(sample.distance_m),
            yaw_rad=float(yaw_rad),
            frame_id=self.frame_id,
        )
        return self._update_result(
            pose=self.pose,
            valid=False,
            reject_reason=reason,
            yaw_rad=yaw_rad,
        )

    @staticmethod
    def _update_result(
        *,
        pose: OpticalFlowPose,
        valid: bool,
        reject_reason: str,
        yaw_rad: float,
        body_delta: tuple[float, float] = (0.0, 0.0),
        map_delta: tuple[float, float] = (0.0, 0.0),
        dt_s: float = 0.0,
        body_velocity: tuple[float, float] = (0.0, 0.0),
        map_velocity: tuple[float, float] = (0.0, 0.0),
        gyro_source: str = "",
    ) -> OpticalFlowUpdate:
        return OpticalFlowUpdate(
            pose=pose,
            valid=valid,
            reject_reason=reject_reason,
            body_dx_m=body_delta[0],
            body_dy_m=body_delta[1],
            map_dx_m=map_delta[0],
            map_dy_m=map_delta[1],
            yaw_used_rad=float(yaw_rad),
            dt_s=float(dt_s),
            body_vx_mps=body_velocity[0],
            body_vy_mps=body_velocity[1],
            map_vx_mps=map_velocity[0],
            map_vy_mps=map_velocity[1],
            gyro_source=gyro_source,
        )


def _finite_optional(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def _rotate_xy(x: float, y: float, yaw_rad: float) -> tuple[float, float]:
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return (
        (cos_yaw * x) - (sin_yaw * y),
        (sin_yaw * x) + (cos_yaw * y),
    )


def _integrate_gyro_buffer(
    buffer: Sequence[tuple[float, float, float, float]],
    start_s: float,
    end_s: float,
    tolerance_s: float,
) -> Optional[tuple[float, float, float]]:
    if end_s <= start_s or len(buffer) < 2:
        return None
    if buffer[0][0] > start_s + tolerance_s:
        return None
    if buffer[-1][0] < end_s - tolerance_s:
        return None

    start_value = _gyro_at(buffer, start_s, tolerance_s)
    end_value = _gyro_at(buffer, end_s, tolerance_s)
    if start_value is None or end_value is None:
        return None
    points = [(start_s, *start_value)]
    points.extend(item for item in buffer if start_s < item[0] < end_s)
    points.append((end_s, *end_value))

    integral = [0.0, 0.0, 0.0]
    for left, right in zip(points, points[1:]):
        dt_s = right[0] - left[0]
        for axis in range(3):
            integral[axis] += 0.5 * (left[axis + 1] + right[axis + 1]) * dt_s
    return integral[0], integral[1], integral[2]


def _gyro_at(
    buffer: Sequence[tuple[float, float, float, float]],
    stamp_s: float,
    tolerance_s: float,
) -> Optional[tuple[float, float, float]]:
    if stamp_s <= buffer[0][0]:
        if buffer[0][0] - stamp_s <= tolerance_s:
            return buffer[0][1], buffer[0][2], buffer[0][3]
        return None
    if stamp_s >= buffer[-1][0]:
        if stamp_s - buffer[-1][0] <= tolerance_s:
            return buffer[-1][1], buffer[-1][2], buffer[-1][3]
        return None
    for left, right in zip(buffer, buffer[1:]):
        if left[0] <= stamp_s <= right[0]:
            interval = right[0] - left[0]
            if interval <= 0.0:
                return None
            ratio = (stamp_s - left[0]) / interval
            return tuple(
                left[axis] + ratio * (right[axis] - left[axis])
                for axis in range(1, 4)
            )
    return None
