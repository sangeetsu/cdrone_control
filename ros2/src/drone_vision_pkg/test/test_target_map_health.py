from pathlib import Path
from types import SimpleNamespace
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from drone_vision_pkg.target_map_node import TargetMapNode


def test_zed_tracking_loss_clears_target_history():
    node = SimpleNamespace(
        now_s=lambda: 100.0,
        memory=SimpleNamespace(tracks={1: object()}),
        require_estimator_valid=True,
        estimator_valid=True,
        last_estimator_s=100.0,
        estimator_timeout_s=2.5,
        last_zed_status_s=0.0,
        zed_tracking_ok=False,
    )
    TargetMapNode.zed_status_callback(node, String(data="OK"))
    assert TargetMapNode.local_source_valid(node)
    TargetMapNode.zed_status_callback(node, String(data="tracking lost"))
    assert not TargetMapNode.local_source_valid(node)
    assert not node.memory.tracks


def test_one_hz_estimator_does_not_clear_valid_local_target_map():
    node = SimpleNamespace(
        now_s=lambda: 100.0, require_estimator_valid=True,
        estimator_valid=True, last_estimator_s=99.0,
        estimator_timeout_s=2.5, zed_tracking_ok=True,
        last_zed_status_s=100.0,
    )
    assert TargetMapNode.local_source_valid(node)
    node.last_estimator_s = 97.0
    assert not TargetMapNode.local_source_valid(node)


def test_local_pose_jump_clears_target_history():
    old_pose = PoseStamped()
    old_pose.pose.orientation.w = 1.0
    new_pose = PoseStamped()
    new_pose.pose.orientation.w = 1.0
    new_pose.pose.position.x = 1.0
    node = SimpleNamespace(
        now_s=lambda: 100.0,
        latest_pose=old_pose,
        last_pose_s=99.9,
        ownship_pose_timeout_s=0.5,
        memory=SimpleNamespace(tracks={1: object()}),
    )
    TargetMapNode.ownship_pose_callback(node, new_pose)
    assert not node.memory.tracks
