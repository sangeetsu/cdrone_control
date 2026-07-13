from __future__ import annotations

import csv
import inspect
import json
import math
from copy import deepcopy
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
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
    configured_ownship_pose_topic,
)
from drone_control_pkg.imu_optical_flow_fusion import (
    FusionUpdate,
    ImuOpticalFlowFusion,
    ImuSample,
)
from drone_control_pkg.minjerk_waypoint_utils import yaw_rad_from_quaternion
from drone_control_pkg.optical_flow_dead_reckon import (
    OpticalFlowDeadReckoner,
    OpticalFlowSample,
    OpticalFlowUpdate,
)
from drone_control_pkg.topic_utils import cdrone_topic, join_topic


CSV_FIELDNAMES = [
    "run_id",
    "stamp_ros_s",
    "wall_time_iso",
    "mission_state",
    "mocap_source_stamp_s",
    "mocap_receive_stamp_s",
    "mocap_age_s",
    "comparison_frame_id",
    "mocap_frame_id",
    "of_frame_id",
    "setpoint_frame_id",
    "frame_match",
    "mocap_x_m",
    "mocap_y_m",
    "mocap_z_m",
    "mocap_yaw_rad",
    "of_x_m",
    "of_y_m",
    "of_z_m",
    "of_yaw_rad",
    "setpoint_x_m",
    "setpoint_y_m",
    "setpoint_z_m",
    "setpoint_yaw_rad",
    "origin_reset_mocap_x_m",
    "origin_reset_mocap_y_m",
    "origin_reset_mocap_z_m",
    "origin_reset_mocap_yaw_rad",
    "yaw_used_rad",
    "err_x_m",
    "err_y_m",
    "err_z_m",
    "err_xy_m",
    "err_3d_m",
    "flow_integrated_x",
    "flow_integrated_y",
    "flow_integrated_xgyro",
    "flow_integrated_ygyro",
    "flow_integrated_zgyro",
    "flow_source_stamp_s",
    "flow_receive_stamp_s",
    "flow_age_s",
    "flow_frame_id",
    "flow_sequence",
    "flow_inter_message_dt_s",
    "flow_integration_coverage_ratio",
    "flow_quality",
    "flow_distance_m",
    "range_m",
    "range_source_stamp_s",
    "range_receive_stamp_s",
    "range_age_s",
    "range_frame_id",
    "flow_dt_s",
    "sample_valid",
    "reject_reason",
    "body_dx_m",
    "body_dy_m",
    "map_dx_m",
    "map_dy_m",
    "imu_frame_id",
    "imu_stamp_s",
    "imu_receive_stamp_s",
    "imu_age_s",
    "imu_yaw_rad",
    "imu_ang_vel_x_radps",
    "imu_ang_vel_y_radps",
    "imu_ang_vel_z_radps",
    "imu_accel_x_mps2",
    "imu_accel_y_mps2",
    "imu_accel_z_mps2",
    "imu_orientation_cov_xx",
    "imu_orientation_cov_yy",
    "imu_orientation_cov_zz",
    "imu_angular_velocity_cov_xx",
    "imu_angular_velocity_cov_yy",
    "imu_angular_velocity_cov_zz",
    "imu_linear_acceleration_cov_xx",
    "imu_linear_acceleration_cov_yy",
    "imu_linear_acceleration_cov_zz",
    "imu_update_valid",
    "imu_update_dt_s",
    "imu_reject_reason",
    "imu_map_accel_x_mps2",
    "imu_map_accel_y_mps2",
    "imu_map_accel_z_mps2",
    "fused_frame_id",
    "fused_x_m",
    "fused_y_m",
    "fused_z_m",
    "fused_vx_mps",
    "fused_vy_mps",
    "fused_vz_mps",
    "fused_yaw_rad",
    "fused_err_x_m",
    "fused_err_y_m",
    "fused_err_z_m",
    "fused_err_xy_m",
    "fused_err_3d_m",
    "fusion_health_state",
    "fusion_cov_x_m2",
    "fusion_cov_y_m2",
    "fusion_cov_z_m2",
    "fusion_cov_vx_m2ps2",
    "fusion_cov_vy_m2ps2",
    "fusion_cov_vz_m2ps2",
    "fusion_cov_bias_ax_m2ps4",
    "fusion_cov_bias_ay_m2ps4",
    "fusion_cov_bias_az_m2ps4",
    "fusion_pose_valid",
    "fusion_imu_valid",
    "fusion_imu_reject_reason",
    "fusion_imu_sensor_age_s",
    "fusion_imu_health_state",
    "fusion_flow_valid",
    "fusion_flow_reject_reason",
    "fusion_flow_sensor_age_s",
    "fusion_flow_health_state",
    "fusion_flow_measurement_variance_x",
    "fusion_flow_measurement_variance_y",
    "fusion_flow_measurement_variance_z",
    "fusion_flow_innovation_x",
    "fusion_flow_innovation_y",
    "fusion_flow_innovation_z",
    "fusion_flow_nis",
    "fusion_flow_gyro_source",
    "fusion_update_type",
    "fusion_reject_reason",
    "fusion_update_dt_s",
    "fusion_body_dx_m",
    "fusion_body_dy_m",
    "fusion_map_dx_m",
    "fusion_map_dy_m",
    "fusion_flow_vx_mps",
    "fusion_flow_vy_mps",
    "imu_only_frame_id",
    "imu_only_x_m",
    "imu_only_y_m",
    "imu_only_z_m",
    "imu_only_vx_mps",
    "imu_only_vy_mps",
    "imu_only_vz_mps",
    "imu_only_err_x_m",
    "imu_only_err_y_m",
    "imu_only_err_z_m",
    "imu_only_err_xy_m",
    "imu_only_err_3d_m",
    "imu_only_pose_valid",
    "imu_only_update_valid",
    "imu_only_reject_reason",
    "imu_only_health_state",
    "of_yaw_source",
]


EVENT_FIELDNAMES = [
    "event_index",
    "event_type",
    "mission_state",
    "source_stamp_s",
    "receive_stamp_s",
    "age_s",
    "frame_id",
    "sequence",
    "valid",
    "reject_reason",
    "health_state",
    "update_type",
    "dt_s",
    "integration_time_s",
    "inter_message_dt_s",
    "integration_coverage_ratio",
    "quality",
    "range_m",
    "measurement_variance_x",
    "measurement_variance_y",
    "measurement_variance_z",
    "innovation_x",
    "innovation_y",
    "innovation_z",
    "nis",
    "gyro_source",
    "x_m",
    "y_m",
    "z_m",
    "yaw_rad",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "covariance_diagonal",
    "linear_acceleration_x",
    "linear_acceleration_y",
    "linear_acceleration_z",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "orientation_covariance",
    "angular_velocity_covariance",
    "linear_acceleration_covariance",
    "integrated_x",
    "integrated_y",
    "integrated_xgyro",
    "integrated_ygyro",
    "integrated_zgyro",
    "flow_covariance",
    "range_variance_m2",
    "detail",
]


ACTIVE_STATES = {
    "SYNC_TAKEOFF_PARAM",
    "SYNC_SPEED_PROFILE",
    "SET_TAKEOFF_MODE",
    "ARMING",
    "REQUEST_TAKEOFF",
    "TAKEOFF",
    "WARMUP_OFFBOARD",
    "SET_OFFBOARD_MODE",
    "WAYPOINTS",
    "WAYPOINT_HOLD",
    "SET_LAND_MODE",
    "WAIT_TOUCHDOWN",
    "DISARMING",
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}


class MocapOfCompareLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("mocap_of_compare_logger_node")

        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("mocap_pose_topic", configured_ownship_pose_topic())
        self.declare_parameter("flow_rad_topic", "")
        self.declare_parameter("flow_range_topic", "")
        self.declare_parameter("imu_topic", "")
        self.declare_parameter("setpoint_topic", "")
        self.declare_parameter("mission_state_topic", "")
        self.declare_parameter("of_pose_topic", "")
        self.declare_parameter("fused_pose_topic", "")
        self.declare_parameter("imu_only_pose_topic", "")
        self.declare_parameter("output_dir", "flight_logs/of_compare")
        self.declare_parameter("run_id", "")
        self.declare_parameter("comparison_frame_id", "map")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("source_best_effort", True)
        self.declare_parameter("quality_min", 10)
        self.declare_parameter("range_min_m", 0.2)
        self.declare_parameter("range_max_m", 5.0)
        self.declare_parameter("gyro_compensation_gain", 1.0)
        self.declare_parameter("flow_scale_x", 1.0)
        self.declare_parameter("flow_scale_y", 1.0)
        self.declare_parameter("reset_on_active_state", True)
        self.declare_parameter("fusion_enabled", True)
        self.declare_parameter("imu_only_enabled", True)
        self.declare_parameter("of_yaw_source", "imu")
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
        self.mocap_pose_topic = str(
            self.get_parameter("mocap_pose_topic").value
        ).strip()
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
        self.setpoint_topic = (
            str(self.get_parameter("setpoint_topic").value).strip()
            or join_topic(self.mavros_namespace, "setpoint_position/local")
        )
        self.mission_state_topic = (
            str(self.get_parameter("mission_state_topic").value).strip()
            or cdrone_topic(self.drone_id, "demo/minjerk_waypoints_state")
        )
        self.of_pose_topic = (
            str(self.get_parameter("of_pose_topic").value).strip()
            or cdrone_topic(self.drone_id, "of_compare/pose")
        )
        self.fused_pose_topic = (
            str(self.get_parameter("fused_pose_topic").value).strip()
            or cdrone_topic(self.drone_id, "of_compare/fused_pose")
        )
        self.imu_only_pose_topic = (
            str(self.get_parameter("imu_only_pose_topic").value).strip()
            or cdrone_topic(self.drone_id, "of_compare/imu_only_pose")
        )
        self.comparison_frame_id = str(
            self.get_parameter("comparison_frame_id").value
        ).strip() or "map"
        self.publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value),
            1.0,
        )
        self.source_best_effort = bool(
            self.get_parameter("source_best_effort").value
        )
        self.reset_on_active_state = bool(
            self.get_parameter("reset_on_active_state").value
        )
        self.fusion_enabled = bool(self.get_parameter("fusion_enabled").value)
        self.imu_only_enabled = bool(
            self.get_parameter("imu_only_enabled").value
        )
        self.of_yaw_source = str(
            self.get_parameter("of_yaw_source").value
        ).strip().lower() or "imu"
        self.run_id = str(self.get_parameter("run_id").value).strip()
        if not self.run_id:
            self.run_id = datetime.now(timezone.utc).strftime(
                "of_compare_%Y%m%dT%H%M%SZ"
            )

        self.run_dir = Path(
            str(self.get_parameter("output_dir").value)
        ).expanduser() / self.run_id
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.run_dir / "of_mocap_compare.csv"
        self.csv_handle = self.csv_path.open(
            "w",
            encoding="utf-8",
            newline="",
        )
        self.csv_writer = csv.DictWriter(
            self.csv_handle,
            fieldnames=CSV_FIELDNAMES,
        )
        self.csv_writer.writeheader()

        self.events_path = self.run_dir / "fusion_events.csv"
        self.events_handle = self.events_path.open(
            "w",
            encoding="utf-8",
            newline="",
        )
        self.events_writer = csv.DictWriter(
            self.events_handle,
            fieldnames=EVENT_FIELDNAMES,
        )
        self.events_writer.writeheader()
        self.event_index = 0
        self.metadata_path = self.run_dir / "run_metadata.json"

        self.dead_reckoner = _construct_compatible(
            OpticalFlowDeadReckoner,
            {
                "frame_id": self.comparison_frame_id,
                "quality_min": int(self.get_parameter("quality_min").value),
                "range_min_m": float(
                    self.get_parameter("range_min_m").value
                ),
                "range_max_m": float(
                    self.get_parameter("range_max_m").value
                ),
                "gyro_compensation_gain": float(
                    self.get_parameter("gyro_compensation_gain").value
                ),
                "flow_scale_x": float(self.get_parameter("flow_scale_x").value),
                "flow_scale_y": float(self.get_parameter("flow_scale_y").value),
                "flow_sensor_yaw_rad": self._float_param(
                    "fusion_flow_sensor_yaw_rad"
                ),
                "max_flow_dt_s": self._float_param("fusion_max_flow_gap_s"),
                "reorder_tolerance_s": self._float_param(
                    "fusion_reorder_tolerance_s"
                ),
                "gyro_coverage_tolerance_s": self._float_param(
                    "fusion_gyro_coverage_tolerance_s"
                ),
                "gyro_buffer_duration_s": self._float_param(
                    "fusion_gyro_buffer_duration_s"
                ),
            },
        )
        estimator_kwargs = {
            "frame_id": self.comparison_frame_id,
            "quality_min": int(self.get_parameter("quality_min").value),
            "range_min_m": float(self.get_parameter("range_min_m").value),
            "range_max_m": float(self.get_parameter("range_max_m").value),
            "gyro_compensation_gain": float(
                self.get_parameter("gyro_compensation_gain").value
            ),
            "flow_scale_x": float(self.get_parameter("flow_scale_x").value),
            "flow_scale_y": float(self.get_parameter("flow_scale_y").value),
            "flow_position_weight": float(
                self.get_parameter("fusion_flow_position_weight").value
            ),
            "flow_velocity_weight": float(
                self.get_parameter("fusion_flow_velocity_weight").value
            ),
            "range_z_weight": float(
                self.get_parameter("fusion_range_z_weight").value
            ),
            "gravity_mps2": float(
                self.get_parameter("fusion_gravity_mps2").value
            ),
            "subtract_gravity": bool(
                self.get_parameter("fusion_subtract_gravity").value
            ),
            "accel_deadband_mps2": float(
                self.get_parameter("fusion_accel_deadband_mps2").value
            ),
            "max_accel_mps2": float(
                self.get_parameter("fusion_max_accel_mps2").value
            ),
            "max_velocity_mps": float(
                self.get_parameter("fusion_max_velocity_mps").value
            ),
            "max_imu_dt_s": float(
                self.get_parameter("fusion_max_imu_dt_s").value
            ),
            "velocity_decay_per_s": float(
                self.get_parameter("fusion_velocity_decay_per_s").value
            ),
            "accel_noise_mps2": self._float_param("fusion_accel_noise_mps2"),
            "accel_bias_rw_mps3": self._float_param(
                "fusion_accel_bias_rw_mps3"
            ),
            "flow_velocity_noise_mps": self._float_param(
                "fusion_flow_velocity_noise_mps"
            ),
            "range_noise_m": self._float_param("fusion_range_noise_m"),
            "accel_lpf_tau_s": self._float_param("fusion_accel_lpf_tau_s"),
            "innovation_gate_nis": self._float_param(
                "fusion_innovation_gate_nis"
            ),
            "range_innovation_gate_nis": self._float_param(
                "fusion_range_innovation_gate_nis"
            ),
            "max_sensor_age_s": self._float_param(
                "fusion_max_sensor_age_s"
            ),
            "reorder_tolerance_s": self._float_param(
                "fusion_reorder_tolerance_s"
            ),
            "max_tilt_rad": self._float_param("fusion_max_tilt_rad"),
            "max_flow_gap_s": self._float_param("fusion_max_flow_gap_s"),
            "flow_gap_noise_scale": self._float_param(
                "fusion_flow_gap_noise_scale"
            ),
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
            "flow_sensor_yaw_rad": self._float_param(
                "fusion_flow_sensor_yaw_rad"
            ),
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
        self.fusion_estimator = _construct_compatible(
            ImuOpticalFlowFusion,
            estimator_kwargs,
        )
        self.imu_only_estimator = _construct_compatible(
            ImuOpticalFlowFusion,
            estimator_kwargs,
        )

        self.latest_mocap: Optional[PoseStamped] = None
        self.latest_mocap_source_stamp_s: Optional[float] = None
        self.latest_mocap_receive_stamp_s: Optional[float] = None
        self.latest_setpoint: Optional[PoseStamped] = None
        self.latest_range: Optional[Range] = None
        self.latest_range_source_stamp_s: Optional[float] = None
        self.latest_range_receive_stamp_s: Optional[float] = None
        self.latest_imu: Optional[Imu] = None
        self.latest_imu_source_stamp_s: Optional[float] = None
        self.latest_imu_receive_stamp_s: Optional[float] = None
        self.latest_imu_yaw_rad: Optional[float] = None
        self.latest_imu_update: Optional[FusionUpdate] = None
        self.latest_imu_only_update: Optional[FusionUpdate] = None
        self.latest_flow: Optional[OpticalFlowSample] = None
        self.latest_flow_source_stamp_s: Optional[float] = None
        self.latest_flow_receive_stamp_s: Optional[float] = None
        self.latest_flow_inter_message_dt_s: Optional[float] = None
        self.latest_flow_frame_id = ""
        self.flow_sequence = 0
        self.latest_update: Optional[OpticalFlowUpdate] = None
        self.latest_fusion_update: Optional[FusionUpdate] = None
        self.latest_fusion_flow_update: Optional[FusionUpdate] = None
        self.latest_state = "IDLE"
        self.was_active = False
        self.estimators_frozen = False
        self.origin_reset_pose: Optional[PoseStamped] = None

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
        source_qos = best_effort_qos if self.source_best_effort else reliable_qos

        self.create_subscription(
            PoseStamped,
            self.mocap_pose_topic,
            self.mocap_callback,
            source_qos,
        )
        self.create_subscription(
            OpticalFlowRad,
            self.flow_rad_topic,
            self.flow_callback,
            best_effort_qos,
        )
        self.create_subscription(
            Range,
            self.flow_range_topic,
            self.range_callback,
            best_effort_qos,
        )
        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            best_effort_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.setpoint_topic,
            self.setpoint_callback,
            reliable_qos,
        )
        self.create_subscription(
            String,
            self.mission_state_topic,
            self.state_callback,
            reliable_qos,
        )

        self.of_pose_pub = self.create_publisher(PoseStamped, self.of_pose_topic, 10)
        self.fused_pose_pub = self.create_publisher(
            PoseStamped,
            self.fused_pose_topic,
            10,
        )
        self.imu_only_pose_pub = self.create_publisher(
            PoseStamped,
            self.imu_only_pose_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Mocap/OF compare logger started: "
            f"mocap={self.mocap_pose_topic} flow={self.flow_rad_topic} "
            f"range={self.flow_range_topic} imu={self.imu_topic} "
            f"fusion_enabled={self.fusion_enabled} "
            f"imu_only_enabled={self.imu_only_enabled} csv={self.csv_path}"
        )
        self.get_logger().warning(
            "fusion_flow_position_weight, fusion_flow_velocity_weight, "
            "and fusion_range_z_weight are deprecated compatibility "
            "arguments; tune EKF covariance/noise and innovation gates instead."
        )
        self._write_metadata()

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def mocap_callback(self, msg: PoseStamped) -> None:
        receive_stamp_s = self.now_s()
        source_stamp_s = self._source_stamp_s(msg, receive_stamp_s)
        self.latest_mocap = deepcopy(msg)
        self.latest_mocap_source_stamp_s = source_stamp_s
        self.latest_mocap_receive_stamp_s = receive_stamp_s
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()
        self._write_event(
            "mocap",
            source_stamp_s=source_stamp_s,
            receive_stamp_s=receive_stamp_s,
            frame_id=msg.header.frame_id,
            pose=msg.pose,
        )

    def range_callback(self, msg: Range) -> None:
        receive_stamp_s = self.now_s()
        source_stamp_s = self._source_stamp_s(msg, receive_stamp_s)
        self.latest_range = deepcopy(msg)
        self.latest_range_source_stamp_s = source_stamp_s
        self.latest_range_receive_stamp_s = receive_stamp_s
        self._write_event(
            "range",
            source_stamp_s=source_stamp_s,
            receive_stamp_s=receive_stamp_s,
            frame_id=msg.header.frame_id,
            range_m=float(msg.range),
        )

    def imu_callback(self, msg: Imu) -> None:
        receive_stamp_s = self.now_s()
        source_stamp_s = self._source_stamp_s(msg, receive_stamp_s)
        self.latest_imu = deepcopy(msg)
        self.latest_imu_source_stamp_s = source_stamp_s
        self.latest_imu_receive_stamp_s = receive_stamp_s
        q = msg.orientation
        self.latest_imu_yaw_rad = yaw_rad_from_quaternion(q.x, q.y, q.z, q.w)
        if (
            (self.fusion_enabled or self.imu_only_enabled)
            and self.fusion_estimator.pose is None
        ):
            self.reset_dead_reckoner()
        sample = _construct_compatible(
            ImuSample,
            {
                "stamp_s": source_stamp_s,
                "source_stamp_s": source_stamp_s,
                "receive_stamp_s": receive_stamp_s,
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
                "orientation_covariance": _covariance_or_none(
                    msg.orientation_covariance
                ),
                "angular_velocity_covariance": _covariance_or_none(
                    msg.angular_velocity_covariance
                ),
                "linear_acceleration_covariance": _covariance_or_none(
                    msg.linear_acceleration_covariance
                ),
            },
        )
        update_imu_gyro = getattr(self.dead_reckoner, "update_imu_gyro", None)
        if callable(update_imu_gyro) and not self.estimators_frozen:
            update_imu_gyro(
                stamp_s=source_stamp_s,
                angular_velocity_x=float(msg.angular_velocity.x),
                angular_velocity_y=float(msg.angular_velocity.y),
                angular_velocity_z=float(msg.angular_velocity.z),
            )
        if not self.estimators_frozen:
            if self.fusion_enabled:
                self.latest_imu_update = self.fusion_estimator.update_imu(sample)
            if self.imu_only_enabled:
                self.latest_imu_only_update = self.imu_only_estimator.update_imu(
                    sample
                )
        self._write_event(
            "imu",
            source_stamp_s=source_stamp_s,
            receive_stamp_s=receive_stamp_s,
            frame_id=msg.header.frame_id,
            update=(
                self.latest_imu_update
                if self.fusion_enabled
                else self.latest_imu_only_update
            ),
            pose=(
                self.fusion_estimator.pose
                if self.fusion_enabled
                else self.imu_only_estimator.pose
            ),
            linear_acceleration=(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ),
            angular_velocity=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ),
            orientation=(q.x, q.y, q.z, q.w),
            orientation_covariance=_covariance_or_none(
                msg.orientation_covariance
            ),
            angular_velocity_covariance=_covariance_or_none(
                msg.angular_velocity_covariance
            ),
            linear_acceleration_covariance=_covariance_or_none(
                msg.linear_acceleration_covariance
            ),
            detail="frozen" if self.estimators_frozen else "",
        )
        self.publish_fused_pose()
        self.publish_imu_only_pose()

    def setpoint_callback(self, msg: PoseStamped) -> None:
        self.latest_setpoint = deepcopy(msg)
        receive_stamp_s = self.now_s()
        self._write_event(
            "setpoint",
            source_stamp_s=self._source_stamp_s(msg, receive_stamp_s),
            receive_stamp_s=receive_stamp_s,
            frame_id=msg.header.frame_id,
            pose=msg.pose,
        )

    def state_callback(self, msg: String) -> None:
        receive_stamp_s = self.now_s()
        state = str(msg.data or "").strip()
        active = state in ACTIVE_STATES
        previous_state = self.latest_state
        self.latest_state = state
        if self.reset_on_active_state and active and not self.was_active:
            self.reset_dead_reckoner()
        if state == "COMPLETE" and previous_state != "COMPLETE":
            self._freeze_estimators()
        self.was_active = active
        self._write_event(
            "mission_state",
            source_stamp_s=receive_stamp_s,
            receive_stamp_s=receive_stamp_s,
            detail=f"{previous_state}->{state}",
        )

    def flow_callback(self, msg: OpticalFlowRad) -> None:
        receive_stamp_s = self.now_s()
        source_stamp_s = self._source_stamp_s(msg, receive_stamp_s)
        previous_flow_stamp_s = self.latest_flow_source_stamp_s
        inter_message_dt_s = None
        if previous_flow_stamp_s is not None:
            inter_message_dt_s = source_stamp_s - previous_flow_stamp_s
        self.flow_sequence += 1
        self.latest_flow_source_stamp_s = source_stamp_s
        self.latest_flow_receive_stamp_s = receive_stamp_s
        self.latest_flow_inter_message_dt_s = inter_message_dt_s
        self.latest_flow_frame_id = str(msg.header.frame_id or "")
        range_m = self._flow_range_m(msg)
        integration_time_s = float(msg.integration_time_us) / 1e6
        sample = _construct_compatible(
            OpticalFlowSample,
            {
                "integrated_x": float(msg.integrated_x),
                "integrated_y": float(msg.integrated_y),
                "integrated_xgyro": float(msg.integrated_xgyro),
                "integrated_ygyro": float(msg.integrated_ygyro),
                "integrated_zgyro": float(msg.integrated_zgyro),
                "quality": int(msg.quality),
                "distance_m": range_m,
                "integration_time_s": integration_time_s,
                "source_stamp_s": source_stamp_s,
                "receive_stamp_s": receive_stamp_s,
                "frame_id": self.latest_flow_frame_id,
                "sequence": self.flow_sequence,
                "flow_covariance": None,
                "range_variance_m2": (
                    self._float_param("fusion_range_noise_m") ** 2
                ),
            },
        )
        self.latest_flow = sample
        if self.latest_mocap is None:
            self._write_flow_event(
                sample,
                update=None,
                reject_reason="no_mocap",
            )
            return
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()
        if self.dead_reckoner.pose is None:
            self._write_flow_event(
                sample,
                update=None,
                reject_reason="not_initialized",
            )
            return
        yaw_rad = self._estimator_yaw_rad()
        if not self.estimators_frozen:
            self.latest_update = self.dead_reckoner.update(sample, yaw_rad=yaw_rad)
        if self.fusion_enabled and not self.estimators_frozen:
            if self.fusion_estimator.pose is None:
                self.reset_dead_reckoner()
            if self.fusion_estimator.pose is not None:
                self.latest_fusion_flow_update = (
                    self.fusion_estimator.update_flow(sample)
                )
                self.latest_fusion_update = self.latest_fusion_flow_update
        self._write_flow_event(
            sample,
            update=self.latest_fusion_flow_update,
            reject_reason="frozen" if self.estimators_frozen else "",
        )
        self.publish_of_pose()
        self.publish_fused_pose()

    def reset_dead_reckoner(self) -> None:
        if self.latest_mocap is None:
            return
        pose = self.latest_mocap.pose
        yaw_rad = self._pose_yaw(self.latest_mocap)
        range_m = None
        if self.latest_range is not None:
            range_m = float(self.latest_range.range)
        frame_id = self.latest_mocap.header.frame_id or self.comparison_frame_id
        self.dead_reckoner.reset(
            x_m=pose.position.x,
            y_m=pose.position.y,
            z_m=pose.position.z,
            yaw_rad=yaw_rad,
            range_m=range_m,
            frame_id=frame_id,
        )
        self.fusion_estimator.reset(
            x_m=pose.position.x,
            y_m=pose.position.y,
            z_m=pose.position.z,
            yaw_rad=yaw_rad,
            range_m=range_m,
            frame_id=frame_id,
        )
        self.imu_only_estimator.reset(
            x_m=pose.position.x,
            y_m=pose.position.y,
            z_m=pose.position.z,
            yaw_rad=yaw_rad,
            range_m=None,
            frame_id=frame_id,
        )
        self._set_estimator_frozen(self.fusion_estimator, False)
        self._set_estimator_frozen(self.imu_only_estimator, False)
        self.estimators_frozen = False
        self.latest_fusion_update = None
        self.latest_fusion_flow_update = None
        self.latest_imu_update = None
        self.latest_imu_only_update = None
        self.origin_reset_pose = deepcopy(self.latest_mocap)
        self.get_logger().info(
            "Reset OF, IMU-only, and IMU+OF origins to mocap pose "
            f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
            f"z={pose.position.z:.3f} frame={frame_id}"
        )
        receive_stamp_s = self.now_s()
        self._write_event(
            "reset",
            source_stamp_s=receive_stamp_s,
            receive_stamp_s=receive_stamp_s,
            frame_id=frame_id,
            pose=self.fusion_estimator.pose,
        )

    def publish_of_pose(self) -> None:
        if self.dead_reckoner.pose is None:
            return
        pose = self.dead_reckoner.pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose.frame_id
        msg.pose.position.x = pose.x_m
        msg.pose.position.y = pose.y_m
        msg.pose.position.z = pose.z_m
        half_yaw = pose.yaw_rad * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.of_pose_pub.publish(msg)

    def publish_fused_pose(self) -> None:
        if not self.fusion_enabled or self.fusion_estimator.pose is None:
            return
        pose = self.fusion_estimator.pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose.frame_id
        msg.pose.position.x = pose.x_m
        msg.pose.position.y = pose.y_m
        msg.pose.position.z = pose.z_m
        half_yaw = pose.yaw_rad * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.fused_pose_pub.publish(msg)

    def publish_imu_only_pose(self) -> None:
        if not self.imu_only_enabled or self.imu_only_estimator.pose is None:
            return
        pose = self.imu_only_estimator.pose
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = pose.frame_id
        msg.pose.position.x = pose.x_m
        msg.pose.position.y = pose.y_m
        msg.pose.position.z = pose.z_m
        half_yaw = pose.yaw_rad * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self.imu_only_pose_pub.publish(msg)

    def timer_callback(self) -> None:
        if self.latest_mocap is None:
            return
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()
        self.csv_writer.writerow(self._csv_row())
        self.csv_handle.flush()
        self.events_handle.flush()
        self.publish_of_pose()
        self.publish_fused_pose()
        self.publish_imu_only_pose()

    def destroy_node(self) -> bool:
        try:
            self.csv_handle.flush()
            self.events_handle.flush()
            self.csv_handle.close()
            self.events_handle.close()
        finally:
            return super().destroy_node()

    def _csv_row(self) -> dict[str, object]:
        mocap = self.latest_mocap
        of_pose = self.dead_reckoner.pose
        setpoint = self.latest_setpoint
        imu = self.latest_imu
        imu_update = self.latest_imu_update
        flow = self.latest_flow
        update = self.latest_update
        fused_pose = self.fusion_estimator.pose if self.fusion_enabled else None
        imu_only_pose = (
            self.imu_only_estimator.pose if self.imu_only_enabled else None
        )
        imu_only_update = self.latest_imu_only_update
        fusion_update = self.latest_fusion_update
        fusion_flow_update = self.latest_fusion_flow_update
        origin = self.origin_reset_pose
        row_stamp_s = self.now_s()
        covariance_diagonal = _covariance_diagonal(
            None,
            self.fusion_estimator,
        )
        fusion_health_update = imu_update or fusion_flow_update

        mocap_yaw = self._pose_yaw(mocap) if mocap is not None else None
        setpoint_yaw = self._pose_yaw(setpoint) if setpoint is not None else None
        frame_match = self._frame_match(mocap, setpoint, of_pose)
        err_x = err_y = err_z = err_xy = err_3d = None
        fused_err_x = fused_err_y = fused_err_z = None
        fused_err_xy = fused_err_3d = None
        imu_only_err_x = imu_only_err_y = imu_only_err_z = None
        imu_only_err_xy = imu_only_err_3d = None
        if mocap is not None and of_pose is not None:
            err_x = of_pose.x_m - mocap.pose.position.x
            err_y = of_pose.y_m - mocap.pose.position.y
            err_z = of_pose.z_m - mocap.pose.position.z
            err_xy = math.hypot(err_x, err_y)
            err_3d = math.sqrt((err_x * err_x) + (err_y * err_y) + (err_z * err_z))
        if mocap is not None and fused_pose is not None:
            fused_err_x = fused_pose.x_m - mocap.pose.position.x
            fused_err_y = fused_pose.y_m - mocap.pose.position.y
            fused_err_z = fused_pose.z_m - mocap.pose.position.z
            fused_err_xy = math.hypot(fused_err_x, fused_err_y)
            fused_err_3d = math.sqrt(
                (fused_err_x * fused_err_x)
                + (fused_err_y * fused_err_y)
                + (fused_err_z * fused_err_z)
            )
        if mocap is not None and imu_only_pose is not None:
            imu_only_err_x = imu_only_pose.x_m - mocap.pose.position.x
            imu_only_err_y = imu_only_pose.y_m - mocap.pose.position.y
            imu_only_err_z = imu_only_pose.z_m - mocap.pose.position.z
            imu_only_err_xy = math.hypot(imu_only_err_x, imu_only_err_y)
            imu_only_err_3d = math.sqrt(
                (imu_only_err_x * imu_only_err_x)
                + (imu_only_err_y * imu_only_err_y)
                + (imu_only_err_z * imu_only_err_z)
            )

        row = {
            "run_id": self.run_id,
            "stamp_ros_s": row_stamp_s,
            "wall_time_iso": datetime.now(timezone.utc).isoformat(),
            "mission_state": self.latest_state,
            "mocap_source_stamp_s": _csv_float(
                self.latest_mocap_source_stamp_s
            ),
            "mocap_receive_stamp_s": _csv_float(
                self.latest_mocap_receive_stamp_s
            ),
            "mocap_age_s": _csv_float(
                _age_s(row_stamp_s, self.latest_mocap_source_stamp_s)
            ),
            "comparison_frame_id": self.comparison_frame_id,
            "mocap_frame_id": mocap.header.frame_id if mocap else "",
            "of_frame_id": of_pose.frame_id if of_pose else "",
            "setpoint_frame_id": setpoint.header.frame_id if setpoint else "",
            "frame_match": int(frame_match),
            "mocap_x_m": _pose_x(mocap),
            "mocap_y_m": _pose_y(mocap),
            "mocap_z_m": _pose_z(mocap),
            "mocap_yaw_rad": _csv_float(mocap_yaw),
            "of_x_m": _csv_float(of_pose.x_m if of_pose else None),
            "of_y_m": _csv_float(of_pose.y_m if of_pose else None),
            "of_z_m": _csv_float(of_pose.z_m if of_pose else None),
            "of_yaw_rad": _csv_float(of_pose.yaw_rad if of_pose else None),
            "setpoint_x_m": _pose_x(setpoint),
            "setpoint_y_m": _pose_y(setpoint),
            "setpoint_z_m": _pose_z(setpoint),
            "setpoint_yaw_rad": _csv_float(setpoint_yaw),
            "origin_reset_mocap_x_m": _pose_x(origin),
            "origin_reset_mocap_y_m": _pose_y(origin),
            "origin_reset_mocap_z_m": _pose_z(origin),
            "origin_reset_mocap_yaw_rad": _csv_float(
                self._pose_yaw(origin) if origin is not None else None
            ),
            "yaw_used_rad": _csv_float(update.yaw_used_rad if update else None),
            "err_x_m": _csv_float(err_x),
            "err_y_m": _csv_float(err_y),
            "err_z_m": _csv_float(err_z),
            "err_xy_m": _csv_float(err_xy),
            "err_3d_m": _csv_float(err_3d),
            "flow_integrated_x": _csv_float(flow.integrated_x if flow else None),
            "flow_integrated_y": _csv_float(flow.integrated_y if flow else None),
            "flow_integrated_xgyro": _csv_float(
                flow.integrated_xgyro if flow else None
            ),
            "flow_integrated_ygyro": _csv_float(
                flow.integrated_ygyro if flow else None
            ),
            "flow_integrated_zgyro": _csv_float(
                flow.integrated_zgyro if flow else None
            ),
            "flow_source_stamp_s": _csv_float(
                self.latest_flow_source_stamp_s
            ),
            "flow_receive_stamp_s": _csv_float(
                self.latest_flow_receive_stamp_s
            ),
            "flow_age_s": _csv_float(
                _age_s(row_stamp_s, self.latest_flow_source_stamp_s)
            ),
            "flow_frame_id": self.latest_flow_frame_id,
            "flow_sequence": self.flow_sequence if flow else "",
            "flow_inter_message_dt_s": _csv_float(
                self.latest_flow_inter_message_dt_s
            ),
            "flow_integration_coverage_ratio": _csv_float(
                _integration_coverage_ratio(
                    flow.integration_time_s if flow else None,
                    self.latest_flow_inter_message_dt_s,
                )
            ),
            "flow_quality": flow.quality if flow else "",
            "flow_distance_m": _csv_float(flow.distance_m if flow else None),
            "range_m": _csv_float(
                float(self.latest_range.range)
                if self.latest_range is not None
                else None
            ),
            "range_source_stamp_s": _csv_float(
                self.latest_range_source_stamp_s
            ),
            "range_receive_stamp_s": _csv_float(
                self.latest_range_receive_stamp_s
            ),
            "range_age_s": _csv_float(
                _age_s(row_stamp_s, self.latest_range_source_stamp_s)
            ),
            "range_frame_id": (
                self.latest_range.header.frame_id if self.latest_range else ""
            ),
            "flow_dt_s": _csv_float(flow.integration_time_s if flow else None),
            "sample_valid": int(bool(update.valid)) if update else 0,
            "reject_reason": update.reject_reason if update else "no_flow",
            "body_dx_m": _csv_float(update.body_dx_m if update else None),
            "body_dy_m": _csv_float(update.body_dy_m if update else None),
            "map_dx_m": _csv_float(update.map_dx_m if update else None),
            "map_dy_m": _csv_float(update.map_dy_m if update else None),
            "imu_frame_id": imu.header.frame_id if imu else "",
            "imu_stamp_s": _csv_float(self.latest_imu_source_stamp_s),
            "imu_receive_stamp_s": _csv_float(
                self.latest_imu_receive_stamp_s
            ),
            "imu_age_s": _csv_float(
                _age_s(row_stamp_s, self.latest_imu_source_stamp_s)
            ),
            "imu_yaw_rad": _csv_float(self.latest_imu_yaw_rad),
            "imu_ang_vel_x_radps": _csv_float(
                imu.angular_velocity.x if imu else None
            ),
            "imu_ang_vel_y_radps": _csv_float(
                imu.angular_velocity.y if imu else None
            ),
            "imu_ang_vel_z_radps": _csv_float(
                imu.angular_velocity.z if imu else None
            ),
            "imu_accel_x_mps2": _csv_float(
                imu.linear_acceleration.x if imu else None
            ),
            "imu_accel_y_mps2": _csv_float(
                imu.linear_acceleration.y if imu else None
            ),
            "imu_accel_z_mps2": _csv_float(
                imu.linear_acceleration.z if imu else None
            ),
            "imu_orientation_cov_xx": _imu_covariance_value(
                imu,
                "orientation_covariance",
                0,
            ),
            "imu_orientation_cov_yy": _imu_covariance_value(
                imu,
                "orientation_covariance",
                4,
            ),
            "imu_orientation_cov_zz": _imu_covariance_value(
                imu,
                "orientation_covariance",
                8,
            ),
            "imu_angular_velocity_cov_xx": _imu_covariance_value(
                imu,
                "angular_velocity_covariance",
                0,
            ),
            "imu_angular_velocity_cov_yy": _imu_covariance_value(
                imu,
                "angular_velocity_covariance",
                4,
            ),
            "imu_angular_velocity_cov_zz": _imu_covariance_value(
                imu,
                "angular_velocity_covariance",
                8,
            ),
            "imu_linear_acceleration_cov_xx": _imu_covariance_value(
                imu,
                "linear_acceleration_covariance",
                0,
            ),
            "imu_linear_acceleration_cov_yy": _imu_covariance_value(
                imu,
                "linear_acceleration_covariance",
                4,
            ),
            "imu_linear_acceleration_cov_zz": _imu_covariance_value(
                imu,
                "linear_acceleration_covariance",
                8,
            ),
            "imu_update_valid": int(bool(imu_update.valid)) if imu_update else 0,
            "imu_update_dt_s": _csv_float(imu_update.dt_s if imu_update else None),
            "imu_reject_reason": (
                imu_update.reject_reason if imu_update else "no_imu"
            ),
            "imu_map_accel_x_mps2": _csv_float(
                imu_update.accel_map_x_mps2 if imu_update else None
            ),
            "imu_map_accel_y_mps2": _csv_float(
                imu_update.accel_map_y_mps2 if imu_update else None
            ),
            "imu_map_accel_z_mps2": _csv_float(
                imu_update.accel_map_z_mps2 if imu_update else None
            ),
            "fused_frame_id": fused_pose.frame_id if fused_pose else "",
            "fused_x_m": _csv_float(fused_pose.x_m if fused_pose else None),
            "fused_y_m": _csv_float(fused_pose.y_m if fused_pose else None),
            "fused_z_m": _csv_float(fused_pose.z_m if fused_pose else None),
            "fused_vx_mps": _csv_float(fused_pose.vx_mps if fused_pose else None),
            "fused_vy_mps": _csv_float(fused_pose.vy_mps if fused_pose else None),
            "fused_vz_mps": _csv_float(fused_pose.vz_mps if fused_pose else None),
            "fused_yaw_rad": _csv_float(
                fused_pose.yaw_rad if fused_pose else None
            ),
            "fused_err_x_m": _csv_float(fused_err_x),
            "fused_err_y_m": _csv_float(fused_err_y),
            "fused_err_z_m": _csv_float(fused_err_z),
            "fused_err_xy_m": _csv_float(fused_err_xy),
            "fused_err_3d_m": _csv_float(fused_err_3d),
            "fusion_health_state": (
                "frozen"
                if self.estimators_frozen
                else _update_text(fusion_health_update, "health_state")
            ),
            "fusion_cov_x_m2": _csv_float(_item(covariance_diagonal, 0)),
            "fusion_cov_y_m2": _csv_float(_item(covariance_diagonal, 1)),
            "fusion_cov_z_m2": _csv_float(_item(covariance_diagonal, 2)),
            "fusion_cov_vx_m2ps2": _csv_float(_item(covariance_diagonal, 3)),
            "fusion_cov_vy_m2ps2": _csv_float(_item(covariance_diagonal, 4)),
            "fusion_cov_vz_m2ps2": _csv_float(_item(covariance_diagonal, 5)),
            "fusion_cov_bias_ax_m2ps4": _csv_float(
                _item(covariance_diagonal, 6)
            ),
            "fusion_cov_bias_ay_m2ps4": _csv_float(
                _item(covariance_diagonal, 7)
            ),
            "fusion_cov_bias_az_m2ps4": _csv_float(
                _item(covariance_diagonal, 8)
            ),
            "fusion_pose_valid": int(fused_pose is not None),
            "fusion_imu_valid": int(bool(imu_update.valid)) if imu_update else 0,
            "fusion_imu_reject_reason": (
                imu_update.reject_reason if imu_update else "no_imu"
            ),
            "fusion_imu_sensor_age_s": _csv_float(
                _update_value(imu_update, "sensor_age_s")
            ),
            "fusion_imu_health_state": _update_text(
                imu_update,
                "health_state",
            ),
            "fusion_flow_valid": (
                int(bool(fusion_flow_update.valid)) if fusion_flow_update else 0
            ),
            "fusion_flow_reject_reason": (
                fusion_flow_update.reject_reason
                if fusion_flow_update
                else "no_flow"
            ),
            "fusion_flow_sensor_age_s": _csv_float(
                _update_value(fusion_flow_update, "sensor_age_s")
            ),
            "fusion_flow_health_state": _update_text(
                fusion_flow_update,
                "health_state",
            ),
            "fusion_flow_measurement_variance_x": _csv_float(
                _update_value(fusion_flow_update, "measurement_variance_x")
            ),
            "fusion_flow_measurement_variance_y": _csv_float(
                _update_value(fusion_flow_update, "measurement_variance_y")
            ),
            "fusion_flow_measurement_variance_z": _csv_float(
                _update_value(fusion_flow_update, "measurement_variance_z")
            ),
            "fusion_flow_innovation_x": _csv_float(
                _update_value(fusion_flow_update, "innovation_x")
            ),
            "fusion_flow_innovation_y": _csv_float(
                _update_value(fusion_flow_update, "innovation_y")
            ),
            "fusion_flow_innovation_z": _csv_float(
                _update_value(fusion_flow_update, "innovation_z")
            ),
            "fusion_flow_nis": _csv_float(
                _update_value(fusion_flow_update, "nis")
            ),
            "fusion_flow_gyro_source": _update_text(
                fusion_flow_update,
                "gyro_source",
            ),
            "fusion_update_type": (
                fusion_update.update_type if fusion_update else ""
            ),
            "fusion_reject_reason": (
                fusion_update.reject_reason if fusion_update else "no_fusion"
            ),
            "fusion_update_dt_s": _csv_float(
                fusion_update.dt_s if fusion_update else None
            ),
            "fusion_body_dx_m": _csv_float(
                fusion_flow_update.body_dx_m if fusion_flow_update else None
            ),
            "fusion_body_dy_m": _csv_float(
                fusion_flow_update.body_dy_m if fusion_flow_update else None
            ),
            "fusion_map_dx_m": _csv_float(
                fusion_flow_update.map_dx_m if fusion_flow_update else None
            ),
            "fusion_map_dy_m": _csv_float(
                fusion_flow_update.map_dy_m if fusion_flow_update else None
            ),
            "fusion_flow_vx_mps": _csv_float(
                fusion_flow_update.flow_vx_mps if fusion_flow_update else None
            ),
            "fusion_flow_vy_mps": _csv_float(
                fusion_flow_update.flow_vy_mps if fusion_flow_update else None
            ),
            "imu_only_frame_id": (
                imu_only_pose.frame_id if imu_only_pose else ""
            ),
            "imu_only_x_m": _csv_float(
                imu_only_pose.x_m if imu_only_pose else None
            ),
            "imu_only_y_m": _csv_float(
                imu_only_pose.y_m if imu_only_pose else None
            ),
            "imu_only_z_m": _csv_float(
                imu_only_pose.z_m if imu_only_pose else None
            ),
            "imu_only_vx_mps": _csv_float(
                imu_only_pose.vx_mps if imu_only_pose else None
            ),
            "imu_only_vy_mps": _csv_float(
                imu_only_pose.vy_mps if imu_only_pose else None
            ),
            "imu_only_vz_mps": _csv_float(
                imu_only_pose.vz_mps if imu_only_pose else None
            ),
            "imu_only_err_x_m": _csv_float(imu_only_err_x),
            "imu_only_err_y_m": _csv_float(imu_only_err_y),
            "imu_only_err_z_m": _csv_float(imu_only_err_z),
            "imu_only_err_xy_m": _csv_float(imu_only_err_xy),
            "imu_only_err_3d_m": _csv_float(imu_only_err_3d),
            "imu_only_pose_valid": int(imu_only_pose is not None),
            "imu_only_update_valid": (
                int(bool(imu_only_update.valid)) if imu_only_update else 0
            ),
            "imu_only_reject_reason": (
                imu_only_update.reject_reason if imu_only_update else "no_imu"
            ),
            "imu_only_health_state": (
                "frozen"
                if self.estimators_frozen
                else _update_text(imu_only_update, "health_state")
            ),
            "of_yaw_source": self.of_yaw_source,
        }
        return row

    def _flow_range_m(self, msg: OpticalFlowRad) -> float:
        if math.isfinite(float(msg.distance)) and float(msg.distance) > 0.0:
            return float(msg.distance)
        if self.latest_range is not None:
            return float(self.latest_range.range)
        return float("nan")

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _write_flow_event(
        self,
        sample: OpticalFlowSample,
        *,
        update: Optional[FusionUpdate],
        reject_reason: str,
    ) -> None:
        inter_message_dt_s = self.latest_flow_inter_message_dt_s
        integration_time_s = float(sample.integration_time_s)
        self._write_event(
            "flow",
            source_stamp_s=self.latest_flow_source_stamp_s,
            receive_stamp_s=self.latest_flow_receive_stamp_s,
            frame_id=self.latest_flow_frame_id,
            sequence=self.flow_sequence,
            update=update,
            pose=self.fusion_estimator.pose,
            reject_reason=reject_reason,
            integration_time_s=integration_time_s,
            inter_message_dt_s=inter_message_dt_s,
            integration_coverage_ratio=_integration_coverage_ratio(
                integration_time_s,
                inter_message_dt_s,
            ),
            quality=int(sample.quality),
            range_m=float(sample.distance_m),
            integrated_flow=(
                sample.integrated_x,
                sample.integrated_y,
                sample.integrated_xgyro,
                sample.integrated_ygyro,
                sample.integrated_zgyro,
            ),
            flow_covariance=getattr(sample, "flow_covariance", None),
            range_variance_m2=getattr(sample, "range_variance_m2", None),
            detail="frozen" if self.estimators_frozen else "",
        )

    def _write_event(
        self,
        event_type: str,
        *,
        source_stamp_s: Optional[float],
        receive_stamp_s: Optional[float],
        frame_id: str = "",
        sequence: object = "",
        update: Optional[FusionUpdate] = None,
        pose: object = None,
        reject_reason: str = "",
        integration_time_s: object = None,
        inter_message_dt_s: object = None,
        integration_coverage_ratio: object = None,
        quality: object = "",
        range_m: object = None,
        linear_acceleration: object = None,
        angular_velocity: object = None,
        orientation: object = None,
        orientation_covariance: object = None,
        angular_velocity_covariance: object = None,
        linear_acceleration_covariance: object = None,
        integrated_flow: object = None,
        flow_covariance: object = None,
        range_variance_m2: object = None,
        detail: str = "",
    ) -> None:
        self.event_index += 1
        resolved_pose = pose or getattr(update, "pose", None)
        valid = getattr(update, "valid", None)
        resolved_reject_reason = reject_reason or str(
            getattr(update, "reject_reason", "") or ""
        )
        if reject_reason and valid is None:
            valid = False
        covariance = _covariance_diagonal(update, self.fusion_estimator)
        row = {
            "event_index": self.event_index,
            "event_type": event_type,
            "mission_state": self.latest_state,
            "source_stamp_s": _csv_float(source_stamp_s),
            "receive_stamp_s": _csv_float(receive_stamp_s),
            "age_s": _csv_float(_age_s(receive_stamp_s, source_stamp_s)),
            "frame_id": str(frame_id or ""),
            "sequence": sequence,
            "valid": "" if valid is None else int(bool(valid)),
            "reject_reason": resolved_reject_reason,
            "health_state": _update_text(update, "health_state"),
            "update_type": _update_text(update, "update_type"),
            "dt_s": _csv_float(_update_value(update, "dt_s")),
            "integration_time_s": _csv_float(integration_time_s),
            "inter_message_dt_s": _csv_float(inter_message_dt_s),
            "integration_coverage_ratio": _csv_float(
                integration_coverage_ratio
            ),
            "quality": quality,
            "range_m": _csv_float(range_m),
            "measurement_variance_x": _csv_float(
                _update_value(update, "measurement_variance_x")
            ),
            "measurement_variance_y": _csv_float(
                _update_value(update, "measurement_variance_y")
            ),
            "measurement_variance_z": _csv_float(
                _update_value(update, "measurement_variance_z")
            ),
            "innovation_x": _csv_float(_update_value(update, "innovation_x")),
            "innovation_y": _csv_float(_update_value(update, "innovation_y")),
            "innovation_z": _csv_float(_update_value(update, "innovation_z")),
            "nis": _csv_float(_update_value(update, "nis")),
            "gyro_source": _update_text(update, "gyro_source"),
            "x_m": _csv_float(_event_pose_value(resolved_pose, "x")),
            "y_m": _csv_float(_event_pose_value(resolved_pose, "y")),
            "z_m": _csv_float(_event_pose_value(resolved_pose, "z")),
            "yaw_rad": _csv_float(_event_pose_yaw(resolved_pose)),
            "vx_mps": _csv_float(_event_pose_value(resolved_pose, "vx")),
            "vy_mps": _csv_float(_event_pose_value(resolved_pose, "vy")),
            "vz_mps": _csv_float(_event_pose_value(resolved_pose, "vz")),
            "covariance_diagonal": json.dumps(covariance),
            "linear_acceleration_x": _csv_float(_item(linear_acceleration, 0)),
            "linear_acceleration_y": _csv_float(_item(linear_acceleration, 1)),
            "linear_acceleration_z": _csv_float(_item(linear_acceleration, 2)),
            "angular_velocity_x": _csv_float(_item(angular_velocity, 0)),
            "angular_velocity_y": _csv_float(_item(angular_velocity, 1)),
            "angular_velocity_z": _csv_float(_item(angular_velocity, 2)),
            "orientation_x": _csv_float(_item(orientation, 0)),
            "orientation_y": _csv_float(_item(orientation, 1)),
            "orientation_z": _csv_float(_item(orientation, 2)),
            "orientation_w": _csv_float(_item(orientation, 3)),
            "orientation_covariance": _json_array(orientation_covariance),
            "angular_velocity_covariance": _json_array(
                angular_velocity_covariance
            ),
            "linear_acceleration_covariance": _json_array(
                linear_acceleration_covariance
            ),
            "integrated_x": _csv_float(_item(integrated_flow, 0)),
            "integrated_y": _csv_float(_item(integrated_flow, 1)),
            "integrated_xgyro": _csv_float(_item(integrated_flow, 2)),
            "integrated_ygyro": _csv_float(_item(integrated_flow, 3)),
            "integrated_zgyro": _csv_float(_item(integrated_flow, 4)),
            "flow_covariance": _json_array(flow_covariance),
            "range_variance_m2": _csv_float(range_variance_m2),
            "detail": detail,
        }
        self.events_writer.writerow(row)
        if self.event_index % 50 == 0 or event_type in {"mission_state", "reset"}:
            self.events_handle.flush()

    def _write_metadata(self) -> None:
        parameters = {
            name: _json_safe(parameter.value)
            for name, parameter in sorted(self._parameters.items())
        }
        metadata = {
            "schema_version": 2,
            "created_at_utc": datetime.now(timezone.utc).isoformat(),
            "run_id": self.run_id,
            "node": self.get_name(),
            "comparison_frame_id": self.comparison_frame_id,
            "observational_only": True,
            "estimators": ["optical_flow_only", "imu_only", "imu_optical_flow_ekf"],
            "topics": {
                "mocap": self.mocap_pose_topic,
                "optical_flow": self.flow_rad_topic,
                "range": self.flow_range_topic,
                "imu": self.imu_topic,
                "setpoint": self.setpoint_topic,
                "mission_state": self.mission_state_topic,
                "of_pose": self.of_pose_topic,
                "fused_pose": self.fused_pose_topic,
                "imu_only_pose": self.imu_only_pose_topic,
            },
            "outputs": {
                "comparison_csv": str(self.csv_path.resolve()),
                "fusion_events_csv": str(self.events_path.resolve()),
                "metadata_json": str(self.metadata_path.resolve()),
            },
            "parameters": parameters,
            "deprecated_parameters": [
                "fusion_flow_position_weight",
                "fusion_flow_velocity_weight",
                "fusion_range_z_weight",
            ],
        }
        temporary_path = self.metadata_path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary_path.replace(self.metadata_path)

    def _freeze_estimators(self) -> None:
        self._set_estimator_frozen(self.fusion_estimator, True)
        self._set_estimator_frozen(self.imu_only_estimator, True)
        self._zero_estimator_velocity(self.fusion_estimator)
        self._zero_estimator_velocity(self.imu_only_estimator)
        self.estimators_frozen = True
        self.get_logger().info(
            "Mission COMPLETE: froze OF, IMU-only, and fused estimates and "
            "zeroed estimator velocity."
        )

    @staticmethod
    def _set_estimator_frozen(estimator: object, frozen: bool) -> None:
        if frozen:
            freeze = getattr(estimator, "freeze", None)
            if callable(freeze):
                freeze()
                return
        else:
            unfreeze = getattr(estimator, "unfreeze", None)
            if callable(unfreeze):
                unfreeze()
                return
        set_frozen = getattr(estimator, "set_frozen", None)
        if callable(set_frozen):
            set_frozen(frozen)

    @staticmethod
    def _zero_estimator_velocity(estimator: object) -> None:
        pose = getattr(estimator, "pose", None)
        if pose is None or not all(
            hasattr(pose, name) for name in ("vx_mps", "vy_mps", "vz_mps")
        ):
            return
        try:
            estimator.pose = replace(
                pose,
                vx_mps=0.0,
                vy_mps=0.0,
                vz_mps=0.0,
            )
        except (TypeError, ValueError, AttributeError):
            return

    def _pose_yaw(self, msg: Optional[PoseStamped]) -> float:
        if msg is None:
            return 0.0
        q = msg.pose.orientation
        return yaw_rad_from_quaternion(q.x, q.y, q.z, q.w)

    def _estimator_yaw_rad(self) -> float:
        if self.of_yaw_source == "mocap" and self.latest_mocap is not None:
            return self._pose_yaw(self.latest_mocap)
        if self.latest_imu_yaw_rad is not None:
            return self.latest_imu_yaw_rad
        if self.dead_reckoner.pose is not None:
            return self.dead_reckoner.pose.yaw_rad
        if self.origin_reset_pose is not None:
            return self._pose_yaw(self.origin_reset_pose)
        return 0.0

    def _source_stamp_s(self, msg: object, fallback_s: float) -> float:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is None:
            return fallback_s
        try:
            value = float(stamp.sec) + (float(stamp.nanosec) / 1e9)
        except (TypeError, ValueError, AttributeError):
            return fallback_s
        return value if math.isfinite(value) and value > 0.0 else fallback_s

    def _stamp_s(self, msg: object) -> float:
        receive_stamp_s = self.now_s()
        return self._source_stamp_s(msg, receive_stamp_s)

    def _frame_match(
        self,
        mocap: Optional[PoseStamped],
        setpoint: Optional[PoseStamped],
        of_pose: object,
    ) -> bool:
        expected = self.comparison_frame_id
        if mocap is None or of_pose is None:
            return False
        if str(mocap.header.frame_id or "").strip() != expected:
            return False
        if str(getattr(of_pose, "frame_id", "") or "").strip() != expected:
            return False
        if setpoint is None:
            return True
        setpoint_frame = str(setpoint.header.frame_id or "").strip()
        return not setpoint_frame or setpoint_frame == expected


def _construct_compatible(factory, values: dict[str, object]):
    """Construct a core object while tolerating an older installed API."""
    parameters = inspect.signature(factory).parameters
    accepts_kwargs = any(
        parameter.kind == inspect.Parameter.VAR_KEYWORD
        for parameter in parameters.values()
    )
    resolved = values if accepts_kwargs else {
        name: value for name, value in values.items() if name in parameters
    }
    return factory(**resolved)


def _covariance_or_none(values: object) -> Optional[tuple[float, ...]]:
    try:
        parsed = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return None
    if not parsed or parsed[0] < 0.0:
        return None
    if not all(math.isfinite(value) for value in parsed):
        return None
    return parsed


def _imu_covariance_value(
    imu: Optional[Imu],
    field_name: str,
    index: int,
) -> str:
    if imu is None:
        return ""
    values = getattr(imu, field_name, None)
    covariance = _covariance_or_none(values)
    return _csv_float(_item(covariance, index))


def _age_s(now_s: object, source_stamp_s: object) -> Optional[float]:
    try:
        now_value = float(now_s)
        source_value = float(source_stamp_s)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(now_value) or not math.isfinite(source_value):
        return None
    return now_value - source_value


def _integration_coverage_ratio(
    integration_time_s: object,
    inter_message_dt_s: object,
) -> Optional[float]:
    try:
        integration_value = float(integration_time_s)
        interval_value = float(inter_message_dt_s)
    except (TypeError, ValueError):
        return None
    if (
        not math.isfinite(integration_value)
        or not math.isfinite(interval_value)
        or interval_value <= 0.0
    ):
        return None
    return integration_value / interval_value


def _update_value(update: object, name: str) -> object:
    if update is None:
        return None
    return getattr(update, name, None)


def _update_text(update: object, name: str, *, default: str = "") -> str:
    if update is None:
        return default
    value = getattr(update, name, default)
    return str(value if value is not None else default)


def _covariance_diagonal(update: object, estimator: object) -> list[float]:
    candidates = [
        getattr(update, "covariance_diagonal", None),
        getattr(estimator, "covariance_diagonal", None),
        getattr(estimator, "covariance", None),
        getattr(estimator, "P", None),
    ]
    for candidate in candidates:
        if callable(candidate):
            try:
                candidate = candidate()
            except TypeError:
                continue
        if candidate is None:
            continue
        if hasattr(candidate, "tolist"):
            candidate = candidate.tolist()
        try:
            values = list(candidate)
        except TypeError:
            continue
        if values and isinstance(values[0], (list, tuple)):
            values = [
                row[index]
                for index, row in enumerate(values)
                if index < len(row)
            ]
        parsed: list[float] = []
        try:
            parsed = [float(value) for value in values]
        except (TypeError, ValueError):
            continue
        if parsed and all(math.isfinite(value) for value in parsed):
            return parsed
    return []


def _item(values: object, index: int) -> object:
    if values is None:
        return None
    try:
        return values[index]
    except (IndexError, KeyError, TypeError):
        return None


def _event_pose_value(pose: object, axis: str) -> object:
    if pose is None:
        return None
    direct_name = f"{axis}_m" if axis in {"x", "y", "z"} else f"{axis}_mps"
    direct = getattr(pose, direct_name, None)
    if direct is not None:
        return direct
    if axis in {"x", "y", "z"}:
        position = getattr(pose, "position", None)
        return getattr(position, axis, None)
    return None


def _event_pose_yaw(pose: object) -> object:
    if pose is None:
        return None
    yaw_rad = getattr(pose, "yaw_rad", None)
    if yaw_rad is not None:
        return yaw_rad
    orientation = getattr(pose, "orientation", None)
    if orientation is None:
        return None
    return yaw_rad_from_quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w,
    )


def _json_array(values: object) -> str:
    if values is None:
        return ""
    try:
        parsed = [float(value) for value in values]
    except (TypeError, ValueError):
        return ""
    return json.dumps(parsed)


def _json_safe(value: object) -> object:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    return str(value)


def _pose_x(msg: Optional[PoseStamped]) -> str:
    return _csv_float(msg.pose.position.x if msg is not None else None)


def _pose_y(msg: Optional[PoseStamped]) -> str:
    return _csv_float(msg.pose.position.y if msg is not None else None)


def _pose_z(msg: Optional[PoseStamped]) -> str:
    return _csv_float(msg.pose.position.z if msg is not None else None)


def _csv_float(value: object) -> str:
    if value is None:
        return ""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(parsed):
        return ""
    return f"{parsed:.9f}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MocapOfCompareLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
