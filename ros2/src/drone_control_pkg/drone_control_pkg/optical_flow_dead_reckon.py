from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class OpticalFlowSample:
    integrated_x: float
    integrated_y: float
    integrated_xgyro: float
    integrated_ygyro: float
    integrated_zgyro: float
    quality: int
    distance_m: float
    integration_time_s: float


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


class OpticalFlowDeadReckoner:
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
    ) -> None:
        self.frame_id = frame_id or "map"
        self.quality_min = int(quality_min)
        self.range_min_m = float(range_min_m)
        self.range_max_m = float(range_max_m)
        self.gyro_compensation_gain = float(gyro_compensation_gain)
        self.flow_scale_x = float(flow_scale_x)
        self.flow_scale_y = float(flow_scale_y)
        self.pose: Optional[OpticalFlowPose] = None
        self.origin_pose: Optional[OpticalFlowPose] = None
        self.origin_range_m: Optional[float] = None

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
        return self.pose

    def update(
        self,
        sample: OpticalFlowSample,
        *,
        yaw_rad: float,
    ) -> OpticalFlowUpdate:
        if self.pose is None:
            pose = OpticalFlowPose(0.0, 0.0, 0.0, yaw_rad, self.frame_id)
            return OpticalFlowUpdate(
                pose=pose,
                valid=False,
                reject_reason="not_initialized",
                body_dx_m=0.0,
                body_dy_m=0.0,
                map_dx_m=0.0,
                map_dy_m=0.0,
                yaw_used_rad=float(yaw_rad),
            )

        reject_reason = self._reject_reason(sample)
        if reject_reason:
            self.pose = OpticalFlowPose(
                x_m=self.pose.x_m,
                y_m=self.pose.y_m,
                z_m=self._z_from_range(sample.distance_m),
                yaw_rad=float(yaw_rad),
                frame_id=self.frame_id,
            )
            return OpticalFlowUpdate(
                pose=self.pose,
                valid=False,
                reject_reason=reject_reason,
                body_dx_m=0.0,
                body_dy_m=0.0,
                map_dx_m=0.0,
                map_dy_m=0.0,
                yaw_used_rad=float(yaw_rad),
            )

        flow_x = sample.integrated_x - self._gyro_compensation(
            sample.integrated_xgyro
        )
        flow_y = sample.integrated_y - self._gyro_compensation(
            sample.integrated_ygyro
        )

        body_dx_m = self.flow_scale_x * sample.distance_m * flow_y
        body_dy_m = self.flow_scale_y * sample.distance_m * (-flow_x)

        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        map_dx_m = (cos_yaw * body_dx_m) - (sin_yaw * body_dy_m)
        map_dy_m = (sin_yaw * body_dx_m) + (cos_yaw * body_dy_m)

        self.pose = OpticalFlowPose(
            x_m=self.pose.x_m + map_dx_m,
            y_m=self.pose.y_m + map_dy_m,
            z_m=self._z_from_range(sample.distance_m),
            yaw_rad=float(yaw_rad),
            frame_id=self.frame_id,
        )
        return OpticalFlowUpdate(
            pose=self.pose,
            valid=True,
            reject_reason="",
            body_dx_m=body_dx_m,
            body_dy_m=body_dy_m,
            map_dx_m=map_dx_m,
            map_dy_m=map_dy_m,
            yaw_used_rad=float(yaw_rad),
        )

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
        if not math.isfinite(float(sample.integrated_x)):
            return "flow_x_not_finite"
        if not math.isfinite(float(sample.integrated_y)):
            return "flow_y_not_finite"
        return ""

    def _gyro_compensation(self, value: float) -> float:
        gain = float(self.gyro_compensation_gain)
        if gain == 0.0:
            return 0.0
        parsed = float(value)
        if not math.isfinite(parsed):
            return 0.0
        return gain * parsed
