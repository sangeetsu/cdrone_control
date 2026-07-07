from __future__ import annotations

import csv
import math
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OpticalFlowRad
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String

from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
    configured_ownship_pose_topic,
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
    "flow_quality",
    "flow_distance_m",
    "range_m",
    "flow_dt_s",
    "sample_valid",
    "reject_reason",
    "body_dx_m",
    "body_dy_m",
    "map_dx_m",
    "map_dy_m",
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
        self.declare_parameter("setpoint_topic", "")
        self.declare_parameter("mission_state_topic", "")
        self.declare_parameter("of_pose_topic", "")
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

        self.dead_reckoner = OpticalFlowDeadReckoner(
            frame_id=self.comparison_frame_id,
            quality_min=int(self.get_parameter("quality_min").value),
            range_min_m=float(self.get_parameter("range_min_m").value),
            range_max_m=float(self.get_parameter("range_max_m").value),
            gyro_compensation_gain=float(
                self.get_parameter("gyro_compensation_gain").value
            ),
            flow_scale_x=float(self.get_parameter("flow_scale_x").value),
            flow_scale_y=float(self.get_parameter("flow_scale_y").value),
        )

        self.latest_mocap: Optional[PoseStamped] = None
        self.latest_setpoint: Optional[PoseStamped] = None
        self.latest_range: Optional[Range] = None
        self.latest_flow: Optional[OpticalFlowSample] = None
        self.latest_update: Optional[OpticalFlowUpdate] = None
        self.latest_state = "IDLE"
        self.was_active = False
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
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Mocap/OF compare logger started: "
            f"mocap={self.mocap_pose_topic} flow={self.flow_rad_topic} "
            f"range={self.flow_range_topic} csv={self.csv_path}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def mocap_callback(self, msg: PoseStamped) -> None:
        self.latest_mocap = deepcopy(msg)
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()

    def range_callback(self, msg: Range) -> None:
        self.latest_range = deepcopy(msg)

    def setpoint_callback(self, msg: PoseStamped) -> None:
        self.latest_setpoint = deepcopy(msg)

    def state_callback(self, msg: String) -> None:
        state = str(msg.data or "").strip()
        active = state in ACTIVE_STATES
        if self.reset_on_active_state and active and not self.was_active:
            self.reset_dead_reckoner()
        self.latest_state = state
        self.was_active = active

    def flow_callback(self, msg: OpticalFlowRad) -> None:
        range_m = self._flow_range_m(msg)
        sample = OpticalFlowSample(
            integrated_x=float(msg.integrated_x),
            integrated_y=float(msg.integrated_y),
            integrated_xgyro=float(msg.integrated_xgyro),
            integrated_ygyro=float(msg.integrated_ygyro),
            integrated_zgyro=float(msg.integrated_zgyro),
            quality=int(msg.quality),
            distance_m=range_m,
            integration_time_s=float(msg.integration_time_us) / 1e6,
        )
        self.latest_flow = sample
        if self.latest_mocap is None:
            return
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()
        yaw_rad = self._pose_yaw(self.latest_mocap)
        self.latest_update = self.dead_reckoner.update(sample, yaw_rad=yaw_rad)
        self.publish_of_pose()

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
        self.origin_reset_pose = deepcopy(self.latest_mocap)
        self.get_logger().info(
            "Reset OF dead-reckon origin to mocap pose "
            f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
            f"z={pose.position.z:.3f} frame={frame_id}"
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

    def timer_callback(self) -> None:
        if self.latest_mocap is None:
            return
        if self.dead_reckoner.pose is None:
            self.reset_dead_reckoner()
        self.csv_writer.writerow(self._csv_row())
        self.csv_handle.flush()
        self.publish_of_pose()

    def destroy_node(self) -> bool:
        try:
            self.csv_handle.flush()
            self.csv_handle.close()
        finally:
            return super().destroy_node()

    def _csv_row(self) -> dict[str, object]:
        mocap = self.latest_mocap
        of_pose = self.dead_reckoner.pose
        setpoint = self.latest_setpoint
        flow = self.latest_flow
        update = self.latest_update
        origin = self.origin_reset_pose

        mocap_yaw = self._pose_yaw(mocap) if mocap is not None else None
        setpoint_yaw = self._pose_yaw(setpoint) if setpoint is not None else None
        frame_match = self._frame_match(mocap, setpoint, of_pose)
        err_x = err_y = err_z = err_xy = err_3d = None
        if mocap is not None and of_pose is not None:
            err_x = of_pose.x_m - mocap.pose.position.x
            err_y = of_pose.y_m - mocap.pose.position.y
            err_z = of_pose.z_m - mocap.pose.position.z
            err_xy = math.hypot(err_x, err_y)
            err_3d = math.sqrt((err_x * err_x) + (err_y * err_y) + (err_z * err_z))

        row = {
            "run_id": self.run_id,
            "stamp_ros_s": self.now_s(),
            "wall_time_iso": datetime.now(timezone.utc).isoformat(),
            "mission_state": self.latest_state,
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
            "flow_quality": flow.quality if flow else "",
            "flow_distance_m": _csv_float(flow.distance_m if flow else None),
            "range_m": _csv_float(
                float(self.latest_range.range)
                if self.latest_range is not None
                else None
            ),
            "flow_dt_s": _csv_float(flow.integration_time_s if flow else None),
            "sample_valid": int(bool(update.valid)) if update else 0,
            "reject_reason": update.reject_reason if update else "no_flow",
            "body_dx_m": _csv_float(update.body_dx_m if update else None),
            "body_dy_m": _csv_float(update.body_dy_m if update else None),
            "map_dx_m": _csv_float(update.map_dx_m if update else None),
            "map_dy_m": _csv_float(update.map_dy_m if update else None),
        }
        return row

    def _flow_range_m(self, msg: OpticalFlowRad) -> float:
        if math.isfinite(float(msg.distance)) and float(msg.distance) > 0.0:
            return float(msg.distance)
        if self.latest_range is not None:
            return float(self.latest_range.range)
        return float("nan")

    def _pose_yaw(self, msg: Optional[PoseStamped]) -> float:
        if msg is None:
            return 0.0
        q = msg.pose.orientation
        return yaw_rad_from_quaternion(q.x, q.y, q.z, q.w)

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
