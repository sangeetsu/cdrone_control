# Copyright 2026 cdrone_control contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import sys
import threading

import pytest
from scipy.spatial.transform import Rotation
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.realsense_tracker_node import (
    frame_pose_matches,
    RealsenseTrackerNode,
    matching_image_stamps,
    normalize_zed_enum_name,
    normalize_zed_flip_mode,
    normalize_zed_image_view,
    rotate_camera_intrinsics_180,
    zed_timestamp_key,
)
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from drone_vision_pkg.projection import RealsenseProjection


def test_ros_color_and_depth_must_share_capture_stamp() -> None:
    color, depth = Image(), Image()
    color.header.stamp.sec = depth.header.stamp.sec = 123
    color.header.stamp.nanosec = depth.header.stamp.nanosec = 456
    assert matching_image_stamps(color, depth)
    depth.header.stamp.nanosec += 1
    assert not matching_image_stamps(color, depth)
    assert not matching_image_stamps(color, None)


def test_ros_projection_rejects_stale_or_different_frame_pose() -> None:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp.sec = 100
    assert frame_pose_matches(100.1, pose, 100.2, "map")
    assert not frame_pose_matches(100.1, pose, 100.8, "map")
    assert not frame_pose_matches(100.4, pose, 100.4, "map")
    assert not frame_pose_matches(100.1, pose, 100.2, "odom")


def test_upside_down_zed_rotation_projects_forward_and_correct_left_right() -> None:
    mounted = Rotation.from_euler("x", 180.0, degrees=True)
    rotated_image = mounted * Rotation.from_euler("x", 180.0, degrees=True)
    projector = RealsenseProjection([0.15, -0.06, -0.03],
                                    list(rotated_image.as_euler("xyz")), "map")
    projector.set_intrinsics(700.0, 700.0, 640.0, 360.0)
    center = projector.pixel_depth_to_body(640.0, 360.0, 5.0)
    right = projector.pixel_depth_to_body(740.0, 360.0, 5.0)
    assert center[0] == pytest.approx(5.15)
    assert center[1] == pytest.approx(-0.06)
    assert right[1] < center[1]


def test_detector_waits_for_stable_fresh_zed_tracking() -> None:
    clock = [100.0]
    node = SimpleNamespace(
        now_s=lambda: clock[0], zed_ready_before_inference_s=5.0,
        zed_ready_since_s=None, last_zed_status_s=0.0,
        zed_tracking_status="missing")
    assert not RealsenseTrackerNode.zed_ready_for_inference(node)
    RealsenseTrackerNode.zed_tracking_status_callback(node, String(data="OK"))
    clock[0] = 104.9
    RealsenseTrackerNode.zed_tracking_status_callback(node, String(data="OK"))
    assert not RealsenseTrackerNode.zed_ready_for_inference(node)
    clock[0] = 105.1
    assert RealsenseTrackerNode.zed_ready_for_inference(node)
    clock[0] = 106.2
    assert not RealsenseTrackerNode.zed_ready_for_inference(node)
    RealsenseTrackerNode.zed_tracking_status_callback(node, String(data="tracking lost"))
    assert node.zed_ready_since_s is None


def test_embedded_zed_handoff_keeps_capture_stamp_and_intrinsics() -> None:
    class Projector:
        def __init__(self):
            self.intrinsics = None

        def has_intrinsics(self):
            return self.intrinsics is not None

        def set_intrinsics(self, **kwargs):
            self.intrinsics = kwargs

    projector = Projector()
    node = SimpleNamespace(
        source_mode="embedded_zed", projector=projector,
        embedded_frame_lock=threading.Lock(), latest_embedded_frame=None)
    color = object()
    depth = object()
    RealsenseTrackerNode.accept_zed_frame(
        node, color, depth, 1_234_567_890, (700.0, 701.0, 639.0, 359.0))
    assert node.latest_embedded_frame == (color, depth, (1, 234_567_890))
    assert projector.intrinsics == {
        "fx": 700.0, "fy": 701.0, "cx": 639.0, "cy": 359.0}


def test_embedded_pose_callback_buffers_motion_without_changing_active_projection() -> None:
    class Projector:
        drone_position_m = None

        def update_pose(self, _msg):
            raise AssertionError("the inference callback owns projection state")

    node = SimpleNamespace(
        source_mode="embedded_zed", projector=Projector(),
        pose_history_lock=threading.Lock(), pose_history=[],
        last_received_pose_position_m=None, last_received_pose_stamp_s=0.0,
        pose_jump_pending=False, last_projection_pose_s=0.0,
        now_s=lambda: 100.0)

    def pose_at(stamp_s: int, x: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp.sec = stamp_s
        pose.pose.position.x = x
        return pose

    first = pose_at(1, 0.0)
    moving = pose_at(2, 1.0)
    jumped = pose_at(3, 5.0)
    for pose in (first, moving):
        RealsenseTrackerNode.pose_callback(node, pose)
    assert node.pose_history == [first, moving]
    assert not node.pose_jump_pending
    RealsenseTrackerNode.pose_callback(node, jumped)
    assert node.pose_history == [jumped]
    assert node.pose_jump_pending


def test_normalize_zed_enum_name_accepts_case_insensitive_values() -> None:
    assert (
        normalize_zed_enum_name(
            "hd720",
            parameter_name="zed_camera_resolution",
            allowed={"HD720", "HD1080"},
        )
        == "HD720"
    )


def test_normalize_zed_enum_name_rejects_unknown_values() -> None:
    with pytest.raises(ValueError, match="zed_depth_mode"):
        normalize_zed_enum_name(
            "fastish",
            parameter_name="zed_depth_mode",
            allowed={"NONE", "PERFORMANCE"},
        )


def test_normalize_zed_flip_mode_accepts_bool_and_aliases() -> None:
    assert normalize_zed_flip_mode(True) == "ON"
    assert normalize_zed_flip_mode(False) == "OFF"
    assert normalize_zed_flip_mode("auto") == "AUTO"
    assert normalize_zed_flip_mode("0") == "OFF"


def test_normalize_zed_image_view_maps_operator_friendly_names() -> None:
    assert normalize_zed_image_view("left") == "LEFT_BGR"
    assert normalize_zed_image_view("right_color") == "RIGHT_BGR"


def test_zed_timestamp_key_splits_nanoseconds() -> None:
    assert zed_timestamp_key(1_234_567_890) == (1, 234_567_890)


def test_rotate_camera_intrinsics_180_updates_principal_point() -> None:
    assert rotate_camera_intrinsics_180(
        width_px=1280,
        height_px=720,
        fx=1160.0,
        fy=1159.0,
        cx=649.7,
        cy=349.5,
    ) == pytest.approx((1160.0, 1159.0, 629.3, 369.5))
