from __future__ import annotations

import numpy as np
import rclpy
from drone_control_pkg.deployment_config import (
    configured_drone_id,
    configured_mavros_namespace,
)
from drone_control_pkg.topic_utils import cdrone_topic, external_pose_input_topic
from drone_msgs.msg import TargetTrack, TargetTrackArray, WorldTargetTrack, WorldTargetTrackArray
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from drone_vision_pkg.target_memory import (
    TRACK_SOURCE_DETECTED,
    TRACK_SOURCE_HELD,
    TRACK_SOURCE_PREDICTED,
    TargetMemory,
    TargetMemoryTrack,
    WorldTrackObservation,
    normalized_quaternion_to_rotation_matrix,
    world_to_body_position,
)


def _stamp_to_float_s(stamp) -> float:
    return float(stamp.sec) + (float(stamp.nanosec) * 1e-9)


def _point_from_array(position_m: np.ndarray) -> Point:
    point = Point()
    point.x = float(position_m[0])
    point.y = float(position_m[1])
    point.z = float(position_m[2])
    return point


def _track_source_name(source: int) -> str:
    if int(source) == TRACK_SOURCE_DETECTED:
        return "detected"
    if int(source) == TRACK_SOURCE_HELD:
        return "held"
    if int(source) == TRACK_SOURCE_PREDICTED:
        return "predicted"
    return "unknown"


class TargetMapNode(Node):
    def __init__(self) -> None:
        super().__init__("target_map_node")

        self.declare_parameter("drone_id", configured_drone_id())
        self.declare_parameter("mavros_namespace", configured_mavros_namespace())
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("world_tracks_topic", "")
        self.declare_parameter("ownship_pose_topic", "")
        self.declare_parameter("target_map_world_topic", "")
        self.declare_parameter("target_map_tracks_topic", "")
        self.declare_parameter("target_map_markers_topic", "")
        self.declare_parameter("association_gate_m", 0.8)
        self.declare_parameter("association_uncertainty_scale", 1.0)
        self.declare_parameter("max_prediction_horizon_s", 3.0)
        self.declare_parameter("prune_after_s", 5.0)
        self.declare_parameter("observation_fresh_s", 0.25)
        self.declare_parameter("confidence_decay_per_s", 0.25)
        self.declare_parameter("observed_position_uncertainty_m", 0.15)
        self.declare_parameter("observed_velocity_uncertainty_mps", 0.35)
        self.declare_parameter("position_process_noise_mps", 0.35)
        self.declare_parameter("velocity_process_noise_mps", 0.25)
        self.declare_parameter("velocity_blend_alpha", 0.55)
        self.declare_parameter("max_velocity_mps", 8.0)
        self.declare_parameter("history_length", 30)
        self.declare_parameter("marker_lifetime_s", 0.5)

        self.drone_id = str(self.get_parameter("drone_id").value)
        self.mavros_namespace = str(self.get_parameter("mavros_namespace").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.world_frame = str(self.get_parameter("world_frame").value).strip() or "map"
        self.observation_fresh_s = float(
            self.get_parameter("observation_fresh_s").value
        )
        self.marker_lifetime_s = float(self.get_parameter("marker_lifetime_s").value)

        self.world_tracks_topic = (
            str(self.get_parameter("world_tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "perception/world_tracks")
        )
        self.ownship_pose_topic = (
            str(self.get_parameter("ownship_pose_topic").value).strip()
            or external_pose_input_topic(self.drone_id)
        )
        self.target_map_world_topic = (
            str(self.get_parameter("target_map_world_topic").value).strip()
            or cdrone_topic(self.drone_id, "target_map/world_tracks")
        )
        self.target_map_tracks_topic = (
            str(self.get_parameter("target_map_tracks_topic").value).strip()
            or cdrone_topic(self.drone_id, "target_map/tracks")
        )
        self.target_map_markers_topic = (
            str(self.get_parameter("target_map_markers_topic").value).strip()
            or cdrone_topic(self.drone_id, "target_map/markers")
        )

        self.memory = TargetMemory(
            association_gate_m=float(self.get_parameter("association_gate_m").value),
            association_uncertainty_scale=float(
                self.get_parameter("association_uncertainty_scale").value
            ),
            max_prediction_horizon_s=float(
                self.get_parameter("max_prediction_horizon_s").value
            ),
            prune_after_s=float(self.get_parameter("prune_after_s").value),
            confidence_decay_per_s=float(
                self.get_parameter("confidence_decay_per_s").value
            ),
            observed_position_uncertainty_m=float(
                self.get_parameter("observed_position_uncertainty_m").value
            ),
            observed_velocity_uncertainty_mps=float(
                self.get_parameter("observed_velocity_uncertainty_mps").value
            ),
            position_process_noise_mps=float(
                self.get_parameter("position_process_noise_mps").value
            ),
            velocity_process_noise_mps=float(
                self.get_parameter("velocity_process_noise_mps").value
            ),
            velocity_blend_alpha=float(
                self.get_parameter("velocity_blend_alpha").value
            ),
            max_velocity_mps=float(self.get_parameter("max_velocity_mps").value),
            history_length=int(self.get_parameter("history_length").value),
        )

        self.latest_pose: PoseStamped | None = None
        self.last_pose_warn_s = 0.0
        self.last_tracks_stamp_s = 0.0

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            WorldTargetTrackArray,
            self.world_tracks_topic,
            self.world_tracks_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            self.ownship_pose_topic,
            self.ownship_pose_callback,
            best_effort_qos,
        )

        self.world_pub = self.create_publisher(
            WorldTargetTrackArray,
            self.target_map_world_topic,
            10,
        )
        self.tracks_pub = self.create_publisher(
            TargetTrackArray,
            self.target_map_tracks_topic,
            10,
        )
        self.markers_pub = self.create_publisher(
            MarkerArray,
            self.target_map_markers_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(self.publish_rate_hz, 1.0),
            self.publish_target_map,
        )

        self.get_logger().info(
            "Target map started: "
            f"world_in={self.world_tracks_topic}, pose={self.ownship_pose_topic}, "
            f"world_out={self.target_map_world_topic}, "
            f"tracks_out={self.target_map_tracks_topic}, "
            f"markers={self.target_map_markers_topic}"
        )

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def ownship_pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def world_tracks_callback(self, msg: WorldTargetTrackArray) -> None:
        now_s = self.now_s()
        observations: list[WorldTrackObservation] = []
        for track in msg.tracks:
            source = int(getattr(track, "source", TRACK_SOURCE_DETECTED))
            last_observed_age_s = max(
                float(getattr(track, "last_observed_age_s", 0.0)),
                0.0,
            )
            detector_track_id = int(
                getattr(track, "detector_track_id", int(track.track_id))
            )
            if detector_track_id == 0 and int(track.track_id) != 0:
                detector_track_id = int(track.track_id)
            observations.append(
                WorldTrackObservation(
                    track_id=int(track.track_id),
                    detector_track_id=detector_track_id,
                    source=source,
                    position_m=np.array(
                        [track.x_m, track.y_m, track.z_m],
                        dtype=np.float64,
                    ),
                    velocity_mps=np.array(
                        [track.vx_mps, track.vy_mps, track.vz_mps],
                        dtype=np.float64,
                    ),
                    confidence=float(track.confidence),
                    bbox_area_px=float(track.bbox_area_px),
                    distance_m=float(track.distance_m),
                    inbound=bool(track.inbound),
                    observed_at_s=now_s - last_observed_age_s,
                    received_at_s=now_s,
                    last_observed_age_s=last_observed_age_s,
                    prediction_horizon_s=float(
                        getattr(track, "prediction_horizon_s", 0.0)
                    ),
                    position_uncertainty_m=float(
                        getattr(track, "position_uncertainty_m", 0.0)
                    ),
                    velocity_uncertainty_mps=float(
                        getattr(track, "velocity_uncertainty_mps", 0.0)
                    ),
                )
            )
        self.memory.update(observations, now_s=now_s)
        self.last_tracks_stamp_s = _stamp_to_float_s(msg.stamp)

    def publish_target_map(self) -> None:
        now_s = self.now_s()
        stamp_msg = self.get_clock().now().to_msg()
        tracks = self.memory.snapshot(
            now_s=now_s,
            observation_fresh_s=self.observation_fresh_s,
        )
        self._publish_world_tracks(tracks, stamp_msg=stamp_msg, now_s=now_s)
        self._publish_body_tracks(tracks, stamp_msg=stamp_msg, now_s=now_s)
        self._publish_markers(tracks, stamp_msg=stamp_msg, now_s=now_s)
        self.memory.prune(now_s)

    def _publish_world_tracks(
        self,
        tracks: list[TargetMemoryTrack],
        *,
        stamp_msg,
        now_s: float,
    ) -> None:
        array = WorldTargetTrackArray()
        array.stamp = stamp_msg
        array.frame_id = self.world_frame
        array.tracks = [
            self._track_to_world_msg(track, stamp_msg=stamp_msg, now_s=now_s)
            for track in tracks
        ]
        self.world_pub.publish(array)

    def _publish_body_tracks(
        self,
        tracks: list[TargetMemoryTrack],
        *,
        stamp_msg,
        now_s: float,
    ) -> None:
        array = TargetTrackArray()
        array.stamp = stamp_msg
        pose = self.latest_pose
        if pose is None:
            if tracks:
                self._warn_pose_missing()
            self.tracks_pub.publish(array)
            return

        ownship_position_m = np.array(
            [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            dtype=np.float64,
        )
        rotation = normalized_quaternion_to_rotation_matrix(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        body_tracks = []
        for track in tracks:
            body_position_m = world_to_body_position(
                track.position_m,
                ownship_position_m,
                rotation,
            )
            body_velocity_mps = rotation.T @ track.velocity_mps
            body_tracks.append(
                self._track_to_body_msg(
                    track,
                    body_position_m=body_position_m,
                    body_velocity_mps=body_velocity_mps,
                    stamp_msg=stamp_msg,
                    now_s=now_s,
                )
            )
        array.tracks = body_tracks
        self.tracks_pub.publish(array)

    def _track_to_world_msg(
        self,
        track: TargetMemoryTrack,
        *,
        stamp_msg,
        now_s: float,
    ) -> WorldTargetTrack:
        age_s = track.observed_age_s(now_s)
        source = track.output_source(
            now_s,
            observation_fresh_s=self.observation_fresh_s,
        )
        msg = WorldTargetTrack()
        msg.stamp = stamp_msg
        msg.track_id = int(track.map_id)
        msg.detector_track_id = int(track.detector_track_id)
        msg.source = int(source)
        msg.x_m = float(track.position_m[0])
        msg.y_m = float(track.position_m[1])
        msg.z_m = float(track.position_m[2])
        msg.vx_mps = float(track.velocity_mps[0])
        msg.vy_mps = float(track.velocity_mps[1])
        msg.vz_mps = float(track.velocity_mps[2])
        msg.distance_m = float(track.distance_m)
        msg.confidence = track.decayed_confidence(
            now_s,
            confidence_decay_per_s=self.memory.confidence_decay_per_s,
        )
        msg.bbox_area_px = float(track.bbox_area_px)
        msg.inbound = bool(track.inbound)
        msg.last_observed_age_s = float(age_s)
        msg.prediction_horizon_s = float(age_s if source == TRACK_SOURCE_PREDICTED else 0.0)
        msg.position_uncertainty_m = float(track.position_uncertainty_m)
        msg.velocity_uncertainty_mps = float(track.velocity_uncertainty_mps)
        return msg

    def _track_to_body_msg(
        self,
        track: TargetMemoryTrack,
        *,
        body_position_m: np.ndarray,
        body_velocity_mps: np.ndarray,
        stamp_msg,
        now_s: float,
    ) -> TargetTrack:
        age_s = track.observed_age_s(now_s)
        source = track.output_source(
            now_s,
            observation_fresh_s=self.observation_fresh_s,
        )
        distance_m = float(np.linalg.norm(body_position_m))
        inbound = False
        if distance_m > 1e-3:
            radial_velocity_mps = -float(
                np.dot(body_position_m, body_velocity_mps)
            ) / distance_m
            inbound = radial_velocity_mps > 0.05
        msg = TargetTrack()
        msg.stamp = stamp_msg
        msg.track_id = int(track.map_id)
        msg.detector_track_id = int(track.detector_track_id)
        msg.source = int(source)
        msg.x_b_m = float(body_position_m[0])
        msg.y_b_m = float(body_position_m[1])
        msg.z_b_m = float(body_position_m[2])
        msg.vx_b_mps = float(body_velocity_mps[0])
        msg.vy_b_mps = float(body_velocity_mps[1])
        msg.vz_b_mps = float(body_velocity_mps[2])
        msg.distance_m = distance_m
        msg.confidence = track.decayed_confidence(
            now_s,
            confidence_decay_per_s=self.memory.confidence_decay_per_s,
        )
        msg.bbox_area_px = float(track.bbox_area_px)
        msg.inbound = bool(inbound)
        msg.last_observed_age_s = float(age_s)
        msg.prediction_horizon_s = float(age_s if source == TRACK_SOURCE_PREDICTED else 0.0)
        msg.position_uncertainty_m = float(track.position_uncertainty_m)
        msg.velocity_uncertainty_mps = float(track.velocity_uncertainty_mps)
        return msg

    def _publish_markers(
        self,
        tracks: list[TargetMemoryTrack],
        *,
        stamp_msg,
        now_s: float,
    ) -> None:
        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.world_frame
        delete_all.header.stamp = stamp_msg
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for track in tracks:
            source = track.output_source(
                now_s,
                observation_fresh_s=self.observation_fresh_s,
            )
            markers.markers.extend(
                [
                    self._position_marker(track, source, stamp_msg=stamp_msg),
                    self._uncertainty_marker(track, source, stamp_msg=stamp_msg),
                    self._path_marker(track, source, stamp_msg=stamp_msg),
                    self._label_marker(track, source, stamp_msg=stamp_msg, now_s=now_s),
                ]
            )
        self.markers_pub.publish(markers)

    def _base_marker(
        self,
        track: TargetMemoryTrack,
        marker_id: int,
        source: int,
        *,
        stamp_msg,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = stamp_msg
        marker.ns = "target_map"
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = int(max(self.marker_lifetime_s, 0.0))
        marker.lifetime.nanosec = int(
            (max(self.marker_lifetime_s, 0.0) % 1.0) * 1e9
        )
        marker.pose.orientation.w = 1.0
        red, green, blue, alpha = self._marker_color(source)
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = alpha
        return marker

    def _position_marker(
        self,
        track: TargetMemoryTrack,
        source: int,
        *,
        stamp_msg,
    ) -> Marker:
        marker = self._base_marker(track, track.map_id * 10, source, stamp_msg=stamp_msg)
        marker.type = Marker.SPHERE
        marker.pose.position = _point_from_array(track.position_m)
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        return marker

    def _uncertainty_marker(
        self,
        track: TargetMemoryTrack,
        source: int,
        *,
        stamp_msg,
    ) -> Marker:
        marker = self._base_marker(
            track,
            (track.map_id * 10) + 1,
            source,
            stamp_msg=stamp_msg,
        )
        marker.type = Marker.SPHERE
        marker.pose.position = _point_from_array(track.position_m)
        diameter_m = max(2.0 * float(track.position_uncertainty_m), 0.05)
        marker.scale.x = diameter_m
        marker.scale.y = diameter_m
        marker.scale.z = diameter_m
        marker.color.a = 0.16 if source == TRACK_SOURCE_PREDICTED else 0.10
        return marker

    def _path_marker(
        self,
        track: TargetMemoryTrack,
        source: int,
        *,
        stamp_msg,
    ) -> Marker:
        marker = self._base_marker(
            track,
            (track.map_id * 10) + 2,
            source,
            stamp_msg=stamp_msg,
        )
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.045
        marker.points = [_point_from_array(position_m) for _, position_m in track.history]
        if len(marker.points) == 1:
            marker.points.append(_point_from_array(track.position_m))
        return marker

    def _label_marker(
        self,
        track: TargetMemoryTrack,
        source: int,
        *,
        stamp_msg,
        now_s: float,
    ) -> Marker:
        marker = self._base_marker(
            track,
            (track.map_id * 10) + 3,
            source,
            stamp_msg=stamp_msg,
        )
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position = _point_from_array(track.position_m + np.array([0.0, 0.0, 0.35]))
        marker.scale.z = 0.24
        marker.text = (
            f"M{track.map_id} {_track_source_name(source)} "
            f"age={track.observed_age_s(now_s):.1f}s"
        )
        marker.color.a = 0.95
        return marker

    def _marker_color(self, source: int) -> tuple[float, float, float, float]:
        if source == TRACK_SOURCE_DETECTED:
            return 0.1, 0.8, 0.25, 0.9
        if source == TRACK_SOURCE_HELD:
            return 0.95, 0.78, 0.1, 0.85
        if source == TRACK_SOURCE_PREDICTED:
            return 1.0, 0.35, 0.05, 0.78
        return 0.7, 0.7, 0.7, 0.7

    def _warn_pose_missing(self) -> None:
        now_s = self.now_s()
        if now_s - self.last_pose_warn_s < 1.0:
            return
        self.last_pose_warn_s = now_s
        self.get_logger().warn(
            "Target map has world tracks but no ownship pose; body-frame "
            f"control tracks are empty until {self.ownship_pose_topic} is live."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetMapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
