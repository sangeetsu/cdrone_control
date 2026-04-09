from __future__ import annotations

import json
from copy import deepcopy
from math import atan2, pi

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import String

from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic, join_topic


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_diff_deg(a_rad: float, b_rad: float) -> float:
    delta = a_rad - b_rad
    while delta > pi:
        delta -= 2.0 * pi
    while delta < -pi:
        delta += 2.0 * pi
    return delta * 180.0 / pi


def _round_or_none(value: float | None, digits: int = 4) -> float | None:
    if value is None:
        return None
    return round(float(value), digits)


class ExternalPoseDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("external_pose_debug_node")

        self.declare_parameter("drone_id", "drone01")
        self.declare_parameter("mavros_namespace", "/mavros")
        self.declare_parameter("source_pose_topic", "/vrpn_mocap/RigidBody3/pose")
        self.declare_parameter("adapter_output_pose_topic", "")
        self.declare_parameter("vision_pose_topic", "")
        self.declare_parameter("local_pose_topic", "")
        self.declare_parameter("global_origin_topic", "")
        self.declare_parameter("home_position_topic", "")
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("history_window_s", 5.0)
        self.declare_parameter("hold_timeout_s", 0.5)
        self.declare_parameter("warn_gap_s", 0.25)
        self.declare_parameter("warn_position_error_m", 0.1)
        self.declare_parameter("warn_yaw_error_deg", 10.0)
        self.declare_parameter("source_best_effort", True)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.source_pose_topic = str(self.get_parameter("source_pose_topic").value).strip()
        self.adapter_output_pose_topic = (
            str(self.get_parameter("adapter_output_pose_topic").value).strip()
            or external_pose_input_topic(self.drone_id)
        )
        self.vision_pose_topic = (
            str(self.get_parameter("vision_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "vision_pose/pose")
        )
        self.local_pose_topic = (
            str(self.get_parameter("local_pose_topic").value).strip()
            or join_topic(self.mavros_namespace, "local_position/pose")
        )
        self.global_origin_topic = (
            str(self.get_parameter("global_origin_topic").value).strip()
            or join_topic(self.mavros_namespace, "global_position/gp_origin")
        )
        self.home_position_topic = (
            str(self.get_parameter("home_position_topic").value).strip()
            or join_topic(self.mavros_namespace, "home_position/home")
        )
        self.publish_rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 0.2)
        self.history_window_s = max(
            float(self.get_parameter("history_window_s").value),
            1.0,
        )
        self.hold_timeout_s = max(float(self.get_parameter("hold_timeout_s").value), 0.0)
        self.warn_gap_s = max(float(self.get_parameter("warn_gap_s").value), 0.0)
        self.warn_position_error_m = max(
            float(self.get_parameter("warn_position_error_m").value),
            0.0,
        )
        self.warn_yaw_error_deg = max(
            float(self.get_parameter("warn_yaw_error_deg").value),
            0.0,
        )
        self.source_best_effort = bool(self.get_parameter("source_best_effort").value)

        self.summary_topic = cdrone_topic(self.drone_id, "external_pose/debug/summary")
        self.held_pose_topic = cdrone_topic(
            self.drone_id, "external_pose/debug/held_source_pose"
        )

        self.source_samples: list[tuple[float, PoseStamped]] = []
        self.adapter_samples: list[tuple[float, PoseStamped]] = []
        self.vision_samples: list[tuple[float, PoseStamped]] = []
        self.local_samples: list[tuple[float, PoseStamped]] = []
        self.latest_home: tuple[float, HomePosition] | None = None
        self.latest_origin: tuple[float, GeoPointStamped] | None = None
        self.last_warning_s = 0.0

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
            self.source_pose_topic,
            self._make_pose_callback(self.source_samples),
            source_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.adapter_output_pose_topic,
            self._make_pose_callback(self.adapter_samples),
            reliable_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.vision_pose_topic,
            self._make_pose_callback(self.vision_samples),
            reliable_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.local_pose_topic,
            self._make_pose_callback(self.local_samples),
            best_effort_qos,
        )
        self.create_subscription(
            GeoPointStamped,
            self.global_origin_topic,
            self._origin_callback,
            reliable_qos,
        )
        self.create_subscription(
            HomePosition,
            self.home_position_topic,
            self._home_callback,
            reliable_qos,
        )

        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.held_pose_pub = self.create_publisher(PoseStamped, self.held_pose_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_debug)

        qos_label = "best_effort" if self.source_best_effort else "reliable"
        self.get_logger().info(
            "External pose debug started: "
            f"{self.source_pose_topic} -> {self.adapter_output_pose_topic} -> "
            f"{self.vision_pose_topic}; local={self.local_pose_topic}; "
            f"source_qos={qos_label}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _make_pose_callback(self, store: list[tuple[float, PoseStamped]]):
        def callback(msg: PoseStamped) -> None:
            store.append((self.now_s(), deepcopy(msg)))
            self._trim_pose_store(store)

        return callback

    def _trim_pose_store(self, store: list[tuple[float, PoseStamped]]) -> None:
        min_keep_s = self.now_s() - max(self.history_window_s, 10.0)
        while len(store) > 1 and store[0][0] < min_keep_s:
            store.pop(0)
        if len(store) > 500:
            del store[:-500]

    def _origin_callback(self, msg: GeoPointStamped) -> None:
        self.latest_origin = (self.now_s(), deepcopy(msg))

    def _home_callback(self, msg: HomePosition) -> None:
        self.latest_home = (self.now_s(), deepcopy(msg))

    def _latest_pose_stats(
        self, store: list[tuple[float, PoseStamped]]
    ) -> dict[str, float] | None:
        if not store:
            return None

        now_s = self.now_s()
        recent = [sample for sample in store if now_s - sample[0] <= self.history_window_s]
        if not recent:
            recent = store[-1:]

        first_time = recent[0][0]
        last_time, last_msg = recent[-1]
        duration = max(last_time - first_time, 1e-6)
        pose = last_msg.pose
        return {
            "count": float(len(recent)),
            "age_s": now_s - last_time,
            "rate_hz": (len(recent) - 1) / duration if len(recent) > 1 else 0.0,
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "yaw_rad": _yaw_from_quaternion(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        }

    def _gap_stats(
        self, store: list[tuple[float, PoseStamped]]
    ) -> tuple[float | None, float | None] | tuple[None, None]:
        now_s = self.now_s()
        recent = [sample for sample in store if now_s - sample[0] <= self.history_window_s]
        if len(recent) < 2:
            return None, None

        gaps = [b[0] - a[0] for a, b in zip(recent, recent[1:])]
        return sum(gaps) / len(gaps), max(gaps)

    def _pose_delta(
        self, source: dict[str, float] | None, target: dict[str, float] | None
    ) -> dict[str, float] | None:
        if source is None or target is None:
            return None
        return {
            "dx_m": target["x"] - source["x"],
            "dy_m": target["y"] - source["y"],
            "dz_m": target["z"] - source["z"],
            "dyaw_deg": _angle_diff_deg(target["yaw_rad"], source["yaw_rad"]),
        }

    def _rounded_pose_stats(
        self, stats: dict[str, float] | None
    ) -> dict[str, float | None] | None:
        if stats is None:
            return None
        return {
            "count": int(stats["count"]),
            "age_s": _round_or_none(stats["age_s"], 3),
            "rate_hz": _round_or_none(stats["rate_hz"], 2),
            "x_m": _round_or_none(stats["x"]),
            "y_m": _round_or_none(stats["y"]),
            "z_m": _round_or_none(stats["z"]),
            "yaw_deg": _round_or_none(stats["yaw_rad"] * 180.0 / pi, 2),
        }

    def _rounded_delta(
        self, delta: dict[str, float] | None
    ) -> dict[str, float | None] | None:
        if delta is None:
            return None
        return {
            "dx_m": _round_or_none(delta["dx_m"]),
            "dy_m": _round_or_none(delta["dy_m"]),
            "dz_m": _round_or_none(delta["dz_m"]),
            "dyaw_deg": _round_or_none(delta["dyaw_deg"], 2),
        }

    def publish_debug(self) -> None:
        now_s = self.now_s()
        source_stats = self._latest_pose_stats(self.source_samples)
        adapter_stats = self._latest_pose_stats(self.adapter_samples)
        vision_stats = self._latest_pose_stats(self.vision_samples)
        local_stats = self._latest_pose_stats(self.local_samples)
        avg_gap_s, max_gap_s = self._gap_stats(self.source_samples)

        summary = {
            "source_pose_topic": self.source_pose_topic,
            "adapter_output_pose_topic": self.adapter_output_pose_topic,
            "vision_pose_topic": self.vision_pose_topic,
            "local_pose_topic": self.local_pose_topic,
            "source_gap_avg_s": _round_or_none(avg_gap_s, 4),
            "source_gap_max_s": _round_or_none(max_gap_s, 4),
            "source": self._rounded_pose_stats(source_stats),
            "adapter": self._rounded_pose_stats(adapter_stats),
            "vision": self._rounded_pose_stats(vision_stats),
            "local": self._rounded_pose_stats(local_stats),
            "source_to_adapter": self._rounded_delta(
                self._pose_delta(source_stats, adapter_stats)
            ),
            "source_to_vision": self._rounded_delta(
                self._pose_delta(source_stats, vision_stats)
            ),
            "source_to_local": self._rounded_delta(
                self._pose_delta(source_stats, local_stats)
            ),
        }

        if self.latest_origin is not None:
            _, origin_msg = self.latest_origin
            summary["global_origin"] = {
                "latitude_deg": _round_or_none(origin_msg.position.latitude, 7),
                "longitude_deg": _round_or_none(origin_msg.position.longitude, 7),
                "altitude_m": _round_or_none(origin_msg.position.altitude, 4),
            }
        else:
            summary["global_origin"] = None

        if self.latest_home is not None:
            _, home_msg = self.latest_home
            summary["home"] = {
                "x_m": _round_or_none(home_msg.position.x),
                "y_m": _round_or_none(home_msg.position.y),
                "z_m": _round_or_none(home_msg.position.z),
                "geo_altitude_m": _round_or_none(home_msg.geo.altitude, 4),
            }
            summary["derived_origin_altitude_m"] = _round_or_none(
                home_msg.geo.altitude - home_msg.position.z,
                4,
            )
            if self.latest_origin is not None:
                _, origin_msg = self.latest_origin
                summary["origin_altitude_error_m"] = _round_or_none(
                    origin_msg.position.altitude
                    - (home_msg.geo.altitude - home_msg.position.z),
                    4,
                )
            if local_stats is not None:
                summary["local_minus_home_z_m"] = _round_or_none(
                    local_stats["z"] - home_msg.position.z
                )
            if source_stats is not None:
                summary["source_minus_home_z_m"] = _round_or_none(
                    source_stats["z"] - home_msg.position.z
                )
        else:
            summary["home"] = None

        summary_msg = String()
        summary_msg.data = json.dumps(summary, sort_keys=True)
        self.summary_pub.publish(summary_msg)

        if (
            self.source_samples
            and self.hold_timeout_s > 0.0
            and source_stats is not None
            and source_stats["age_s"] <= self.hold_timeout_s
        ):
            held_pose = deepcopy(self.source_samples[-1][1])
            held_pose.header.stamp = self.get_clock().now().to_msg()
            self.held_pose_pub.publish(held_pose)

        self._maybe_warn(now_s, source_stats, max_gap_s, local_stats, source_stats)

    def _maybe_warn(
        self,
        now_s: float,
        source_stats: dict[str, float] | None,
        max_gap_s: float | None,
        local_stats: dict[str, float] | None,
        reference_stats: dict[str, float] | None,
    ) -> None:
        warn_parts: list[str] = []
        if source_stats is None:
            warn_parts.append("no source pose samples")
        elif self.warn_gap_s > 0.0 and source_stats["age_s"] > self.warn_gap_s:
            warn_parts.append(f"source age {source_stats['age_s']:.2f}s")

        if max_gap_s is not None and self.warn_gap_s > 0.0 and max_gap_s > self.warn_gap_s:
            warn_parts.append(f"max source gap {max_gap_s:.2f}s")

        source_to_local = self._pose_delta(reference_stats, local_stats)
        if source_to_local is not None:
            pos_error = max(
                abs(source_to_local["dx_m"]),
                abs(source_to_local["dy_m"]),
                abs(source_to_local["dz_m"]),
            )
            yaw_error = abs(source_to_local["dyaw_deg"])
            if (
                self.warn_position_error_m > 0.0
                and pos_error > self.warn_position_error_m
            ):
                detail = (
                    "source->local pos error "
                    f"{pos_error:.3f}m "
                    f"(dx={source_to_local['dx_m']:+.3f}, "
                    f"dy={source_to_local['dy_m']:+.3f}, "
                    f"dz={source_to_local['dz_m']:+.3f})"
                )
                if (
                    self.latest_home is not None
                    and source_stats is not None
                    and local_stats is not None
                ):
                    _, home_msg = self.latest_home
                    home_z_m = float(home_msg.position.z)
                    detail += (
                        f" src_z={source_stats['z']:.3f}"
                        f" local_z={local_stats['z']:.3f}"
                        f" src_minus_home={source_stats['z'] - home_z_m:+.3f}"
                        f" local_minus_home={local_stats['z'] - home_z_m:+.3f}"
                    )
                warn_parts.append(detail)
            if (
                self.warn_position_error_m > 0.0
                and abs(source_to_local["dz_m"]) > self.warn_position_error_m
            ):
                detail = f"source->local vertical mismatch dz={source_to_local['dz_m']:+.3f}m"
                if (
                    self.latest_home is not None
                    and source_stats is not None
                    and local_stats is not None
                ):
                    _, home_msg = self.latest_home
                    home_z_m = float(home_msg.position.z)
                    detail += (
                        f" src_minus_home={source_stats['z'] - home_z_m:+.3f}"
                        f" local_minus_home={local_stats['z'] - home_z_m:+.3f}"
                    )
                warn_parts.append(detail)
            if self.warn_yaw_error_deg > 0.0 and yaw_error > self.warn_yaw_error_deg:
                warn_parts.append(f"source->local yaw error {yaw_error:.1f}deg")

        if not warn_parts:
            return
        if now_s - self.last_warning_s < 1.0:
            return

        self.last_warning_s = now_s
        self.get_logger().warn("; ".join(warn_parts))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExternalPoseDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
