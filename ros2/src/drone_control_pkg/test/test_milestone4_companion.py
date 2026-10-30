"""Controller checks in an isolated ROS domain; no executor or hardware is run."""

from concurrent.futures import Future
from types import SimpleNamespace

import pytest
import rclpy
from rcl_interfaces.msg import ParameterType, ParameterValue

from drone_control_pkg.follow_utils import TrackSnapshot
from drone_control_pkg.milestone3_demo_sequence_node import Milestone3DemoSequenceNode


@pytest.fixture
def controller(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "213")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    settings = dict(local_odom_mode="true", enable_flight="false", companion_only="true",
                    takeoff_altitude_m="2.0", use_speed_profile="false",
                    enable_perimeter_guard="false", require_companion_active="false",
                    required_completion_count="2")
    args = ["--ros-args"]
    for key, value in settings.items():
        args += ["-p", f"{key}:={value}"]
    rclpy.init(args=args)
    node = Milestone3DemoSequenceNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("mismatch", [None, "COM_RC_IN_MODE", "NAV_DLL_ACT", "COM_OBL_RC_ACT"])
def test_companion_profile_is_verified_read_only(controller, mismatch):
    node = controller
    assert node.ZED_PROFILE["COM_RC_IN_MODE"] == 4
    assert node.ZED_PROFILE["NAV_DLL_ACT"] == 0
    assert node.ZED_PROFILE["COM_OBL_RC_ACT"] == 4
    assert node.ZED_PROFILE["COM_OF_LOSS_T"] == 1.0
    # The shared profile used by other launches must not be changed.
    assert "COM_RC_IN_MODE" not in Milestone3DemoSequenceNode.ZED_PROFILE
    values = []
    for name, expected in node.ZED_PROFILE.items():
        actual = expected + 1 if name == mismatch else expected
        values.append(ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=actual)
                      if isinstance(actual, float) else
                      ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=actual))
    future = Future()
    future.set_result(SimpleNamespace(values=values))
    requests = []
    node.param_get_client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: (requests.append(request), future)[1])
    node.param_set_client = SimpleNamespace()  # Any attempted write fails the test.
    node.latest_state.connected = True
    node.service_vision_profile()
    node.service_vision_profile()
    assert len(requests) == 1
    assert node.vision_profile_ok is (mismatch is None)
    if mismatch:
        assert mismatch in node.vision_profile_reason


def test_start_readiness_rejects_stale_velocity_and_disabled_flight(controller):
    node = controller
    node.latest_state.connected = True
    node.mode_client = node.arm_client = SimpleNamespace(wait_for_service=lambda **kw: True)
    node.state_fresh = node.pose_fresh = lambda: True
    node.local_health_reason = lambda: ""
    node.last_perception_time_s = node.now_s()
    node.local_preflight_ready_since_s = node.now_s() - 3.0
    node.last_local_velocity_time_s = node.now_s() - 0.6
    assert "velocity stale" in node.startable()[1]
    node.last_local_velocity_time_s = node.now_s()
    assert node.startable(check_flight=False) == (True, "ready")
    assert "flight disabled" in node.startable()[1]


def test_each_completion_counts_once_and_return_starts_at_required_count(controller):
    node = controller
    node.demo_state = "FOLLOW"
    node.control_block_reason = lambda: ""
    node.clamp_command_to_perimeter_altitude = lambda command: command
    node.command_projection_violation_reason = lambda command: None
    node.publish_follow_command = lambda command: None
    node.has_takeoff_return_target = lambda: True
    now = [100.0]
    node.now_s = lambda: now[0]
    target = TrackSnapshot(7, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.9, 200.0, False, 100.0)
    node.select_target = lambda time: target
    for track_id, expected_state in [(7, "SEARCH"), (7, "SEARCH"), (9, "RETURN_TO_START")]:
        target.track_id = track_id
        node.demo_state = "FOLLOW"
        node.dwell_started_s = now[0] - node.dwell_time_s
        node.step_state_machine()
        assert node.demo_state == expected_state
        now[0] += 5.0
    assert node.completed_track_ids == {7, 9}


def test_hover_blocker_is_visible_in_existing_engagement_message(controller):
    node = controller
    node.demo_state = "STAGE_HOVER"
    node.pose_fresh = lambda: True
    node.current_altitude_m = lambda: 1.85
    node.last_local_velocity_time_s = node.now_s()
    messages = []
    node.state_pub = SimpleNamespace(publish=messages.append)
    node.publish_engagement_state(now_s=node.now_s(), target=None)
    assert "altitude error" in messages[0].blocked_reason
    assert not messages[0].autonomy_ready
