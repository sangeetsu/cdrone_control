from __future__ import annotations

import inspect
import json
import math
from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OpticalFlowRad
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.imu_optical_flow_fusion import (
    FusionPose,
    FusionUpdate,
    ImuOpticalFlowFusion,
    ImuSample,
)
from drone_control_pkg.minjerk_waypoint_utils import yaw_rad_from_quaternion
from drone_control_pkg.optical_flow_dead_reckon import OpticalFlowSample
from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic, join_topic


@dataclass(frozen=True)
class EstimatorHealth:
    ready: bool
    reason: str


def source_stamp_s(msg: object, fallback_s: float) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return fallback_s
    try:
        value = float(stamp.sec) + (float(stamp.nanosec) / 1e9)
    except (TypeError, ValueError, AttributeError):
        return fallback_s
    return value if math.isfinite(value) and value > 0.0 else fallback_s


def covariance_or_none(values: object) -> Optional[tuple[float, ...]]:
    try:
        parsed = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if not parsed or parsed[0] < 0.0:
        return None
    if not all(math.isfinite(value) for value in parsed):
        return None
    return parsed


def range_is_valid(range_m: object, *, range_min_m: float, range_max_m: float) -> bool:
    try:
        value = float(range_m)
    except (TypeError, ValueError):
        return False
    return math.isfinite(value) and range_min_m <= value <= range_max_m


def flow_range_m(msg: object, latest_range: Optional[object]) -> float:
    try:
        distance = float(getattr(msg, "distance"))
    except (TypeError, ValueError, AttributeError):
        distance = float("nan")
    if math.isfinite(distance) and distance > 0.0:
        return distance
    if latest_range is not None:
        try:
            return float(getattr(latest_range, "range"))
        except (TypeError, ValueError, AttributeError):
            return float("nan")
    return float("nan")


def effective_flow_range_m(
    raw_range_m: float,
    *,
    range_min_m: float,
    near_ground_range_m: float,
    allow_near_ground_flow: bool,
) -> float:
    try:
        raw_value = float(raw_range_m)
    except (TypeError, ValueError):
        raw_value = float("nan")
    if math.isfinite(raw_value) and raw_value >= range_min_m:
        return raw_value
    if allow_near_ground_flow and math.isfinite(raw_value) and raw_value >= 0.0:
        return max(float(range_min_m), float(near_ground_range_m))
    return raw_value


def flow_gyro_value(value: object, *, allow_missing_flow_gyro: bool) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        parsed = float("nan")
    if math.isfinite(parsed):
        return parsed
    return 0.0 if allow_missing_flow_gyro else parsed


def flow_gyro_missing(value: object) -> bool:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return True
    return not math.isfinite(parsed)


def make_imu_sample(msg: Imu, *, source_s: float, receive_s: float) -> ImuSample:
    q = msg.orientation
    return _construct_compatible(
        ImuSample,
        {
            "stamp_s": source_s,
            "source_stamp_s": source_s,
            "receive_stamp_s": receive_s,
            "frame_id": str(msg.header.frame_id or ""),
            "orientation_x": float(q.x),
            "orientation_y": float(q.y),
            "orientation_z": float(q.z),
            "orientation_w": float(q.w),
            "angular_velocity_x": float(msg.angular_velocity.x),
            "angular_velocity_y": float(msg.angular_velocity.y),
            "angular_velocity_z": float(msg.angular_velocity.z),
            "linear_acceleration_x": float(msg.linear_acceleration.x),
            "linear_acceleration_y": float(msg.linear_acceleration.y),
            "linear_acceleration_z": float(msg.linear_acceleration.z),
            "orientation_covariance": covariance_or_none(
                msg.orientation_covariance
            ),
            "angular_velocity_covariance": covariance_or_none(
                msg.angular_velocity_covariance
            ),
            "linear_acceleration_covariance": covariance_or_none(
                msg.linear_acceleration_covariance
            ),
        },
    )


def make_flow_sample(
    msg: OpticalFlowRad,
    *,
    range_m: float,
    source_s: float,
    receive_s: float,
    sequence: int,
    range_variance_m2: Optional[float],
    allow_missing_flow_gyro: bool = False,
) -> OpticalFlowSample:
    return _construct_compatible(
        OpticalFlowSample,
        {
            "integrated_x": float(msg.integrated_x),
            "integrated_y": float(msg.integrated_y),
            "integrated_xgyro": flow_gyro_value(
                msg.integrated_xgyro,
                allow_missing_flow_gyro=allow_missing_flow_gyro,
            ),
            "integrated_ygyro": flow_gyro_value(
                msg.integrated_ygyro,
                allow_missing_flow_gyro=allow_missing_flow_gyro,
            ),
            "integrated_zgyro": flow_gyro_value(
                msg.integrated_zgyro,
                allow_missing_flow_gyro=allow_missing_flow_gyro,
            ),
            "quality": int(msg.quality),
            "distance_m": float(range_m),
            "integration_time_s": float(msg.integration_time_us) / 1e6,
            "source_stamp_s": source_s,
            "receive_stamp_s": receive_s,
            "frame_id": str(msg.header.frame_id or ""),
            "sequence": int(sequence),
            "flow_covariance": None,
            "range_variance_m2": range_variance_m2,
        },
    )


def pose_to_msg(pose: FusionPose, stamp, *, frame_id: str = "") -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id or pose.frame_id
    msg.pose.position.x = pose.x_m
    msg.pose.position.y = pose.y_m
    msg.pose.position.z = pose.z_m
    half_yaw = pose.yaw_rad * 0.5
    msg.pose.orientation.z = math.sin(half_yaw)
    msg.pose.orientation.w = math.cos(half_yaw)
    return msg


def evaluate_health(
    *,
    now_s: float,
    initialized: bool,
    last_imu_receive_s: Optional[float],
    last_range_receive_s: Optional[float],
    last_flow_receive_s: Optional[float],
    last_valid_flow_receive_s: Optional[float],
    last_flow_quality: Optional[int],
    latest_range_m: Optional[float],
    latest_flow_update: Optional[FusionUpdate],
    init_time_s: Optional[float],
    max_imu_age_s: float,
    max_range_age_s: float,
    max_flow_age_s: float,
    quality_min: int,
    range_min_m: float,
    range_max_m: float,
    allow_imu_only_warmup: bool,
    imu_only_warmup_s: float,
) -> EstimatorHealth:
    if not initialized:
        return EstimatorHealth(False, "not_initialized")
    if last_imu_receive_s is None or now_s - last_imu_receive_s > max_imu_age_s:
        return EstimatorHealth(False, "imu_stale")
    if last_range_receive_s is None or now_s - last_range_receive_s > max_range_age_s:
        return EstimatorHealth(False, "range_stale")
    if not range_is_valid(
        latest_range_m,
        range_min_m=range_min_m,
        range_max_m=range_max_m,
    ):
        return EstimatorHealth(False, "range_invalid")
    if last_flow_quality is None or int(last_flow_quality) < int(quality_min):
        return EstimatorHealth(False, "flow_quality_below_min")
    if (
        last_valid_flow_receive_s is not None
        and now_s - last_valid_flow_receive_s <= max_flow_age_s
    ):
        return EstimatorHealth(True, "ready")
    if last_flow_receive_s is None or now_s - last_flow_receive_s > max_flow_age_s:
        return EstimatorHealth(False, "flow_stale")
    if latest_flow_update is not None and latest_flow_update.valid:
        return EstimatorHealth(True, "ready")
    if allow_imu_only_warmup and init_time_s is not None:
        if now_s - init_time_s >= imu_only_warmup_s:
            return EstimatorHealth(True, "imu_only_warmup")
    reason = (
        latest_flow_update.reject_reason
        if latest_flow_update is not None and latest_flow_update.reject_reason
        else "flow_not_accepted"
    )
    return EstimatorHealth(False, reason)


class OpticalFlowHoverEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("optical_flow_hover_estimator_node")

        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("flow_rad_topic", "")
        self.declare_parameter("flow_range_topic", "")
        self.declare_parameter("imu_topic", "")
        self.declare_parameter("external_pose_topic", "")
        self.declare_parameter("debug_pose_topic", "")
        self.declare_parameter("status_topic", "")
        self.declare_parameter("source_best_effort", True)
        self.declare_parameter("quality_min", 10)
        self.declare_parameter("range_min_m", 0.2)
        self.declare_parameter("range_max_m", 5.0)
        self.declare_parameter("max_imu_age_s", 0.25)
        self.declare_parameter("max_range_age_s", 0.25)
        self.declare_parameter("max_flow_age_s", 0.50)
        self.declare_parameter("allow_near_ground_flow", True)
        self.declare_parameter("near_ground_range_m", 0.05)
        self.declare_parameter("allow_missing_flow_gyro", True)
        self.declare_parameter("allow_imu_only_warmup", True)
        self.declare_parameter("imu_only_warmup_s", 1.0)
        self.declare_parameter("gyro_compensation_gain", 1.0)
        self.declare_parameter("flow_scale_x", 1.0)
        self.declare_parameter("flow_scale_y", 1.0)
        self.declare_parameter("fusion_flow_position_weight", 0.75)
        self.declare_parameter("fusion_flow_velocity_weight", 0.50)
        self.declare_parameter("fusion_range_z_weight", 0.80)
        self.declare_parameter("fusion_subtract_gravity", True)
        self.declare_parameter("fusion_gravity_mps2", 9.80665)
        self.declare_parameter("fusion_accel_deadband_mps2", 0.05)
        self.declare_parameter("fusion_max_accel_mps2", 6.0)
        self.declare_parameter("fusion_max_velocity_mps", 4.0)
        self.declare_parameter("fusion_max_imu_dt_s", 0.10)
        self.declare_parameter("fusion_velocity_decay_per_s", 0.04)
        self.declare_parameter("fusion_accel_noise_mps2", 0.80)
        self.declare_parameter("fusion_accel_bias_rw_mps3", 0.03)
        self.declare_parameter("fusion_flow_velocity_noise_mps", 0.20)
        self.declare_parameter("fusion_range_noise_m", 0.08)
        self.declare_parameter("fusion_accel_lpf_tau_s", 0.08)
        self.declare_parameter("fusion_innovation_gate_nis", 9.21)
        self.declare_parameter("fusion_range_innovation_gate_nis", 6.635)
        self.declare_parameter("fusion_max_sensor_age_s", 0.25)
        self.declare_parameter("fusion_reorder_tolerance_s", 0.02)
        self.declare_parameter("fusion_max_tilt_rad", 0.7853981633974483)
        self.declare_parameter("fusion_max_flow_gap_s", 0.50)
        self.declare_parameter("fusion_flow_gap_noise_scale", 0.25)
        self.declare_parameter("fusion_flow_quality_noise_scale", 3.0)
        self.declare_parameter("fusion_flow_range_noise_scale", 1.0)
        self.declare_parameter("fusion_gyro_fallback_noise_scale", 2.0)
        self.declare_parameter("fusion_gyro_coverage_tolerance_s", 0.005)
        self.declare_parameter("fusion_gyro_buffer_duration_s", 1.0)
        self.declare_parameter("fusion_flow_sensor_yaw_rad", 0.0)
        self.declare_parameter("fusion_initial_position_std_m", 0.25)
        self.declare_parameter("fusion_initial_velocity_std_mps", 0.50)
        self.declare_parameter("fusion_initial_accel_bias_std_mps2", 0.30)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.map_frame = str(self.get_parameter("map_frame").value).strip() or "map"
        self.quality_min = int(self.get_parameter("quality_min").value)
        self.range_min_m = float(self.get_parameter("range_min_m").value)
        self.range_max_m = float(self.get_parameter("range_max_m").value)
        self.max_imu_age_s = float(self.get_parameter("max_imu_age_s").value)
        self.max_range_age_s = float(self.get_parameter("max_range_age_s").value)
        self.max_flow_age_s = float(self.get_parameter("max_flow_age_s").value)
        self.allow_near_ground_flow = bool(
            self.get_parameter("allow_near_ground_flow").value
        )
        self.near_ground_range_m = float(
            self.get_parameter("near_ground_range_m").value
        )
        self.allow_missing_flow_gyro = bool(
            self.get_parameter("allow_missing_flow_gyro").value
        )
        self.allow_imu_only_warmup = bool(
            self.get_parameter("allow_imu_only_warmup").value
        )
        self.imu_only_warmup_s = float(self.get_parameter("imu_only_warmup_s").value)

        self.flow_rad_topic = (
            str(self.get_parameter("flow_rad_topic").value).strip()
            or join_topic(self.mavros_namespace, "px4flow/raw/optical_flow_rad")
        )
        self.flow_range_topic = (
            str(self.get_parameter("flow_range_topic").value).strip()
            or join_topic(self.mavros_namespace, "px4flow/ground_distance")
        )
        self.imu_topic = (
            str(self.get_parameter("imu_topic").value).strip()
            or join_topic(self.mavros_namespace, "imu/data")
        )
        self.external_pose_topic = (
            str(self.get_parameter("external_pose_topic").value).strip()
            or external_pose_input_topic(self.drone_id)
        )
        self.debug_pose_topic = (
            str(self.get_parameter("debug_pose_topic").value).strip()
            or cdrone_topic(self.drone_id, "of_hover/fused_pose")
        )
        self.status_topic = (
            str(self.get_parameter("status_topic").value).strip()
            or cdrone_topic(self.drone_id, "of_hover/status")
        )

        self.estimator = _construct_compatible(
            ImuOpticalFlowFusion,
            self._estimator_kwargs(),
        )
        self.latest_range: Optional[Range] = None
        self.latest_range_m: Optional[float] = None
        self.latest_effective_flow_range_m: Optional[float] = None
        self.latest_imu_yaw_rad: Optional[float] = None
        self.latest_imu_update: Optional[FusionUpdate] = None
        self.latest_flow_update: Optional[FusionUpdate] = None
        self.latest_flow_gyro_fallback = False
        self.last_imu_receive_s: Optional[float] = None
        self.last_range_receive_s: Optional[float] = None
        self.last_flow_receive_s: Optional[float] = None
        self.last_valid_flow_receive_s: Optional[float] = None
        self.last_flow_quality: Optional[int] = None
        self.flow_sequence = 0
        self.init_time_s: Optional[float] = None
        self.last_health = EstimatorHealth(False, "not_initialized")

        best_effort_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        reliable_qos = qos.QoSProfile(
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        source_qos = (
            best_effort_qos
            if bool(self.get_parameter("source_best_effort").value)
            else reliable_qos
        )

        self.create_subscription(OpticalFlowRad, self.flow_rad_topic, self.flow_callback, source_qos)
        self.create_subscription(Range, self.flow_range_topic, self.range_callback, source_qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, source_qos)
        self.external_pose_pub = self.create_publisher(PoseStamped, self.external_pose_topic, 10)
        self.debug_pose_pub = self.create_publisher(PoseStamped, self.debug_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_loop)

        self.get_logger().info(
            "Optical-flow hover estimator started: "
            f"flow={self.flow_rad_topic} range={self.flow_range_topic} "
            f"imu={self.imu_topic} external_pose={self.external_pose_topic}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def range_callback(self, msg: Range) -> None:
        self.latest_range = deepcopy(msg)
        self.latest_range_m = float(msg.range)
        self.last_range_receive_s = self.now_s()
        self._try_initialize()

    def imu_callback(self, msg: Imu) -> None:
        receive_s = self.now_s()
        source_s = source_stamp_s(msg, receive_s)
        self.last_imu_receive_s = receive_s
        q = msg.orientation
        self.latest_imu_yaw_rad = yaw_rad_from_quaternion(q.x, q.y, q.z, q.w)
        self._try_initialize()
        if self.estimator.pose is None:
            return
        self.latest_imu_update = self.estimator.update_imu(
            make_imu_sample(msg, source_s=source_s, receive_s=receive_s)
        )
        self.publish_debug_pose()

    def flow_callback(self, msg: OpticalFlowRad) -> None:
        receive_s = self.now_s()
        source_s = source_stamp_s(msg, receive_s)
        self.last_flow_receive_s = receive_s
        self.last_flow_quality = int(msg.quality)
        self.flow_sequence += 1
        self._try_initialize()
        if self.estimator.pose is None:
            return

        range_m = flow_range_m(msg, self.latest_range)
        effective_range_m = effective_flow_range_m(
            range_m,
            range_min_m=self.range_min_m,
            near_ground_range_m=self.near_ground_range_m,
            allow_near_ground_flow=self.allow_near_ground_flow,
        )
        self.latest_effective_flow_range_m = effective_range_m
        sample = make_flow_sample(
            msg,
            range_m=effective_range_m,
            source_s=source_s,
            receive_s=receive_s,
            sequence=self.flow_sequence,
            range_variance_m2=self._float_param("fusion_range_noise_m") ** 2,
            allow_missing_flow_gyro=self.allow_missing_flow_gyro,
        )
        self.latest_flow_gyro_fallback = (
            self.allow_missing_flow_gyro
            and (
                flow_gyro_missing(msg.integrated_xgyro)
                or flow_gyro_missing(msg.integrated_ygyro)
            )
        )
        self.latest_flow_update = self.estimator.update_flow(sample)
        if self.latest_flow_update.valid:
            self.last_valid_flow_receive_s = receive_s
        self.publish_debug_pose()

    def publish_loop(self) -> None:
        self.last_health = self._evaluate_health()
        self.publish_status()
        self.publish_debug_pose()
        if not self.last_health.ready or self.estimator.pose is None:
            return
        msg = pose_to_msg(
            self.estimator.pose,
            self.get_clock().now().to_msg(),
            frame_id=self.map_frame,
        )
        self.external_pose_pub.publish(msg)

    def publish_debug_pose(self) -> None:
        if self.estimator.pose is None:
            return
        msg = pose_to_msg(
            self.estimator.pose,
            self.get_clock().now().to_msg(),
            frame_id=self.map_frame,
        )
        self.debug_pose_pub.publish(msg)

    def publish_status(self) -> None:
        pose = self.estimator.pose
        payload = {
            "ready": self.last_health.ready,
            "reason": self.last_health.reason,
            "initialized": pose is not None,
            "last_flow_quality": self.last_flow_quality,
            "range_m": self.latest_range_m,
            "effective_flow_range_m": self.latest_effective_flow_range_m,
            "flow_gyro_fallback": self.latest_flow_gyro_fallback,
            "x_m": pose.x_m if pose is not None else None,
            "y_m": pose.y_m if pose is not None else None,
            "z_m": pose.z_m if pose is not None else None,
            "health_state": (
                self.latest_flow_update.health_state
                if self.latest_flow_update is not None
                else (
                    self.latest_imu_update.health_state
                    if self.latest_imu_update is not None
                    else ""
                )
            ),
            "flow_reject_reason": (
                self.latest_flow_update.reject_reason
                if self.latest_flow_update is not None
                else ""
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def _try_initialize(self) -> None:
        if self.estimator.pose is not None:
            return
        if self.latest_imu_yaw_rad is None:
            return
        range_m = self.latest_range_m
        z_m = range_m if range_is_valid(
            range_m,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
        ) else 0.0
        self.estimator.reset(
            x_m=0.0,
            y_m=0.0,
            z_m=z_m,
            yaw_rad=self.latest_imu_yaw_rad,
            range_m=range_m,
            frame_id=self.map_frame,
        )
        self.init_time_s = self.now_s()
        self.get_logger().info(
            "Optical-flow estimator origin reset from ground: "
            f"x=0.000 y=0.000 z={z_m:.3f} yaw={self.latest_imu_yaw_rad:.3f}"
        )

    def _evaluate_health(self) -> EstimatorHealth:
        health_range_m = self.latest_range_m
        if not range_is_valid(
            health_range_m,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
        ):
            health_range_m = self.latest_effective_flow_range_m
        return evaluate_health(
            now_s=self.now_s(),
            initialized=self.estimator.pose is not None,
            last_imu_receive_s=self.last_imu_receive_s,
            last_range_receive_s=self.last_range_receive_s,
            last_flow_receive_s=self.last_flow_receive_s,
            last_valid_flow_receive_s=self.last_valid_flow_receive_s,
            last_flow_quality=self.last_flow_quality,
            latest_range_m=health_range_m,
            latest_flow_update=self.latest_flow_update,
            init_time_s=self.init_time_s,
            max_imu_age_s=self.max_imu_age_s,
            max_range_age_s=self.max_range_age_s,
            max_flow_age_s=self.max_flow_age_s,
            quality_min=self.quality_min,
            range_min_m=self.range_min_m,
            range_max_m=self.range_max_m,
            allow_imu_only_warmup=self.allow_imu_only_warmup,
            imu_only_warmup_s=self.imu_only_warmup_s,
        )

    def _estimator_kwargs(self) -> dict[str, object]:
        return {
            "frame_id": self.map_frame,
            "quality_min": self.quality_min,
            "range_min_m": self.range_min_m,
            "range_max_m": self.range_max_m,
            "gyro_compensation_gain": self._float_param("gyro_compensation_gain"),
            "flow_scale_x": self._float_param("flow_scale_x"),
            "flow_scale_y": self._float_param("flow_scale_y"),
            "flow_position_weight": self._float_param("fusion_flow_position_weight"),
            "flow_velocity_weight": self._float_param("fusion_flow_velocity_weight"),
            "range_z_weight": self._float_param("fusion_range_z_weight"),
            "gravity_mps2": self._float_param("fusion_gravity_mps2"),
            "subtract_gravity": bool(
                self.get_parameter("fusion_subtract_gravity").value
            ),
            "accel_deadband_mps2": self._float_param("fusion_accel_deadband_mps2"),
            "max_accel_mps2": self._float_param("fusion_max_accel_mps2"),
            "max_velocity_mps": self._float_param("fusion_max_velocity_mps"),
            "max_imu_dt_s": self._float_param("fusion_max_imu_dt_s"),
            "velocity_decay_per_s": self._float_param("fusion_velocity_decay_per_s"),
            "accel_noise_mps2": self._float_param("fusion_accel_noise_mps2"),
            "accel_bias_rw_mps3": self._float_param("fusion_accel_bias_rw_mps3"),
            "flow_velocity_noise_mps": self._float_param(
                "fusion_flow_velocity_noise_mps"
            ),
            "range_noise_m": self._float_param("fusion_range_noise_m"),
            "accel_lpf_tau_s": self._float_param("fusion_accel_lpf_tau_s"),
            "innovation_gate_nis": self._float_param("fusion_innovation_gate_nis"),
            "range_innovation_gate_nis": self._float_param(
                "fusion_range_innovation_gate_nis"
            ),
            "max_sensor_age_s": self._float_param("fusion_max_sensor_age_s"),
            "reorder_tolerance_s": self._float_param("fusion_reorder_tolerance_s"),
            "max_tilt_rad": self._float_param("fusion_max_tilt_rad"),
            "max_flow_gap_s": self._float_param("fusion_max_flow_gap_s"),
            "flow_gap_noise_scale": self._float_param("fusion_flow_gap_noise_scale"),
            "flow_quality_noise_scale": self._float_param(
                "fusion_flow_quality_noise_scale"
            ),
            "flow_range_noise_scale": self._float_param(
                "fusion_flow_range_noise_scale"
            ),
            "gyro_fallback_noise_scale": self._float_param(
                "fusion_gyro_fallback_noise_scale"
            ),
            "gyro_coverage_tolerance_s": self._float_param(
                "fusion_gyro_coverage_tolerance_s"
            ),
            "gyro_buffer_duration_s": self._float_param(
                "fusion_gyro_buffer_duration_s"
            ),
            "flow_sensor_yaw_rad": self._float_param("fusion_flow_sensor_yaw_rad"),
            "initial_position_std_m": self._float_param(
                "fusion_initial_position_std_m"
            ),
            "initial_velocity_std_mps": self._float_param(
                "fusion_initial_velocity_std_mps"
            ),
            "initial_accel_bias_std_mps2": self._float_param(
                "fusion_initial_accel_bias_std_mps2"
            ),
        }

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)


def _construct_compatible(factory, values: dict[str, object]):
    parameters = inspect.signature(factory).parameters
    accepts_kwargs = any(
        parameter.kind == inspect.Parameter.VAR_KEYWORD
        for parameter in parameters.values()
    )
    resolved = values if accepts_kwargs else {
        name: value for name, value in values.items() if name in parameters
    }
    return factory(**resolved)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpticalFlowHoverEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
