"""Milestone 4 preflight recovery and estimator telemetry checks."""

from types import SimpleNamespace
from concurrent.futures import Future
import math

import pytest

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import EstimatorStatus
from std_msgs.msg import String

from drone_control_pkg.milestone3_demo_sequence_node import Milestone3DemoSequenceNode


def pose(z):
    msg = PoseStamped()
    msg.pose.position.z = z
    msg.pose.orientation.w = 1.0
    return msg


def fake_pose_node(state="IDLE"):
    node = SimpleNamespace(
        local_odom_mode=True, last_pose_time_s=99.9, latest_pose=pose(0.0),
        demo_state=state, TERMINAL_STATES={"IDLE", "COMPLETE", "ABORT"},
        local_pose_discontinuous=False, local_pose_fault_until_s=0.0,
        local_preflight_ready_since_s=98.0, local_preflight_dwell_s=2.0,
        now_s=lambda: 100.0, pose_fresh=lambda: True,
        current_yaw_rad=lambda: 0.0, aborts=[],
    )
    node.enter_abort = node.aborts.append
    return node


def test_table_height_is_not_a_pose_jump_once_pose_is_stable():
    node = fake_pose_node()
    node.latest_pose = pose(0.95)
    Milestone3DemoSequenceNode.pose_callback(node, pose(0.96))
    assert not node.local_pose_discontinuous
    assert node.local_pose_fault_until_s == 0.0


def test_preflight_pose_jump_settles_but_in_flight_jump_aborts():
    node = fake_pose_node()
    Milestone3DemoSequenceNode.pose_callback(node, pose(0.95))
    assert not node.local_pose_discontinuous
    assert node.local_pose_fault_until_s == 102.0
    assert node.local_preflight_ready_since_s is None
    assert not node.aborts

    flying = fake_pose_node("FOLLOW")
    Milestone3DemoSequenceNode.pose_callback(flying, pose(0.95))
    assert flying.local_pose_discontinuous
    assert flying.aborts == ["local odometry discontinuity"]


def test_local_frame_change_resets_preflight_readiness():
    node = fake_pose_node()
    node.latest_pose.header.frame_id = "map"
    new = pose(0.0)
    new.header.frame_id = "odom"
    Milestone3DemoSequenceNode.pose_callback(node, new)
    assert node.local_pose_fault_until_s == 102.0
    assert node.local_preflight_ready_since_s is None
    assert not node.local_pose_discontinuous


def test_one_hz_estimator_is_fresh_and_invalid_flags_are_explained():
    current = [100.0]
    node = SimpleNamespace(
        now_s=lambda: current[0], vision_profile_ok=True,
        local_pose_discontinuous=False, local_pose_fault_until_s=0.0,
        last_zed_status_s=100.0, zed_status="OK", last_vision_time_s=100.0,
        last_estimator_time_s=99.0, estimator_timeout_s=2.5,
        estimator_ok=False, estimator_reason="missing",
    )
    msg = EstimatorStatus(
        attitude_status_flag=True, velocity_horiz_status_flag=True,
        velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
        pos_vert_abs_status_flag=True)
    Milestone3DemoSequenceNode.estimator_callback(node, msg)
    node.last_estimator_time_s = 99.0  # Message received one second ago.
    assert Milestone3DemoSequenceNode.local_health_reason(node) == ""
    node.last_estimator_time_s = 97.0
    assert "stale" in Milestone3DemoSequenceNode.local_health_reason(node)
    msg.velocity_horiz_status_flag = False
    Milestone3DemoSequenceNode.estimator_callback(node, msg)
    assert "horizontal velocity invalid" in Milestone3DemoSequenceNode.local_health_reason(node)
    msg.velocity_horiz_status_flag = True
    Milestone3DemoSequenceNode.estimator_callback(node, msg)
    node.local_pose_fault_until_s = 101.0
    assert "waiting to settle" in Milestone3DemoSequenceNode.local_health_reason(node)
    current[0] = 102.0
    node.last_zed_status_s = 102.0
    node.last_vision_time_s = 102.0
    Milestone3DemoSequenceNode.estimator_callback(node, msg)
    assert Milestone3DemoSequenceNode.local_health_reason(node) == ""
    node.local_pose_discontinuous = True
    assert "restart required" in Milestone3DemoSequenceNode.local_health_reason(node)


def test_preflight_requires_continuous_health_after_recovery():
    current = [100.0]
    node = SimpleNamespace(
        local_odom_mode=True, demo_state="IDLE",
        TERMINAL_STATES={"IDLE", "COMPLETE", "ABORT"},
        local_preflight_ready_since_s=None,
        latest_state=SimpleNamespace(connected=True, armed=False),
        estop_latched=False, last_perception_time_s=100.0,
        require_detector_target_on_start=True,
        last_valid_detector_target_time_s=100.0,
        now_s=lambda: current[0], state_fresh=lambda: True,
        pose_fresh=lambda: True, local_health_reason=lambda: "",
        local_velocity_reason=lambda: "",
    )
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s == 100.0
    current[0] = 101.0
    node.local_health_reason = lambda: "PX4 estimator: horizontal position invalid"
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s is None
    current[0] = 102.0
    node.local_health_reason = lambda: ""
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s == 102.0


def test_milestone4_requests_milestone1_estimator_stream_rate():
    requests = []
    future = Future()
    future.set_result(SimpleNamespace(success=True))
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda request: (requests.append(request), future)[1],
    )
    node = SimpleNamespace(
        local_odom_mode=True, latest_state=SimpleNamespace(connected=True),
        now_s=lambda: 100.0, estimator_interval_future=None,
        estimator_interval_started_s=0.0,
        estimator_interval_retry_after_s=0.0,
        estimator_interval_attempts=0, estimator_interval_client=client,
    )
    Milestone3DemoSequenceNode.service_estimator_stream(node)
    assert len(requests) == 1
    assert requests[0].message_id == 230
    assert requests[0].message_rate == 5.0
    Milestone3DemoSequenceNode.service_estimator_stream(node)
    assert node.estimator_interval_attempts == 3


def test_no_target_still_allows_preflight_health_dwell():
    node = SimpleNamespace(
        local_odom_mode=True, demo_state="IDLE",
        TERMINAL_STATES={"IDLE", "COMPLETE", "ABORT"},
        local_preflight_ready_since_s=None,
        latest_state=SimpleNamespace(connected=True, armed=False),
        estop_latched=False, last_perception_time_s=100.0,
        require_detector_target_on_start=False,
        last_valid_detector_target_time_s=0.0,
        now_s=lambda: 100.0, state_fresh=lambda: True,
        pose_fresh=lambda: True, local_health_reason=lambda: "",
        local_velocity_reason=lambda: "",
    )
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s == 100.0


def test_stable_hover_requires_altitude_and_measured_velocity():
    velocity = TwistStamped()
    node = SimpleNamespace(
        now_s=lambda: 100.0, pose_fresh=lambda: True,
        last_local_velocity_time_s=100.0, latest_local_velocity=velocity,
        current_altitude_m=lambda: 2.0, takeoff_target_altitude_m=lambda: 2.0,
        altitude_tolerance_m=0.1,
        stable_hover_max_horizontal_speed_mps=0.25,
        stable_hover_max_vertical_speed_mps=0.15,
    )
    bind_hover_methods(node)
    assert Milestone3DemoSequenceNode.local_hover_stable(node)
    velocity.twist.linear.x = 0.3
    assert not Milestone3DemoSequenceNode.local_hover_stable(node)
    velocity.twist.linear.x = 0.0
    velocity.twist.linear.z = 0.2
    assert not Milestone3DemoSequenceNode.local_hover_stable(node)
    velocity.twist.linear.z = 0.0
    node.current_altitude_m = lambda: 1.8
    assert not Milestone3DemoSequenceNode.local_hover_stable(node)


def test_stage_hover_waits_for_two_continuous_stable_seconds_then_searches():
    now = [100.0]
    stable = [False]
    states = []
    node = SimpleNamespace(
        now_s=lambda: now[0], demo_state="STAGE_HOVER", local_odom_mode=True,
        stable_hover_since_s=None, stage_hover_duration_s=2.0,
        local_hover_stable=lambda: stable[0],
        publish_zero_command=lambda: None,
        publish_takeoff_hold_command=lambda: None,
        last_hover_progress_log_s=100.0, hover_progress=lambda: "waiting",
        get_logger=lambda: SimpleNamespace(info=lambda msg: None),
        set_demo_state=lambda state, reason="": states.append(state),
    )
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert not states
    stable[0] = True
    now[0] = 101.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    now[0] = 102.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert not states
    stable[0] = False
    now[0] = 102.1
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert node.stable_hover_since_s is None
    stable[0] = True
    now[0] = 103.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    now[0] = 105.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert states == ["SEARCH"]


def test_no_target_local_takeoff_reaches_hover_without_detector_track():
    clock = [100.0]
    position_setpoints, velocity_zeros, arm_requests, velocity_holds = [], [], [], []
    node = SimpleNamespace(
        now_s=lambda: clock[0], demo_state="LOCAL_WARMUP", local_odom_mode=True,
        stage_started_s=100.0, offboard_warmup_s=1.0,
        publish_local_position_setpoint=lambda: position_setpoints.append(True),
        publish_zero_command=lambda: velocity_zeros.append(True),
        publish_takeoff_hold_command=lambda: velocity_holds.append(True),
        latest_state=SimpleNamespace(armed=False), offboard_mode="OFFBOARD",
        mode_matches=lambda mode: True, arm_zero_throttle_hold_s=0.0,
        arm_request_sent=False, request_arm=lambda arm: arm_requests.append(arm),
        takeoff_altitude_reached=lambda: True,
    )
    node.stage_elapsed_s = lambda: clock[0] - node.stage_started_s
    def transition(state, reason=""):
        node.demo_state = state
        node.stage_started_s = clock[0]
    node.set_demo_state = transition

    clock[0] = 101.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    Milestone3DemoSequenceNode.step_state_machine(node)
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert arm_requests == [True]
    node.latest_state.armed = True
    Milestone3DemoSequenceNode.step_state_machine(node)
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert node.demo_state == "LOCAL_HANDOFF"
    clock[0] = 102.0
    Milestone3DemoSequenceNode.step_state_machine(node)
    assert node.demo_state == "STAGE_HOVER"
    assert len(position_setpoints) == 5
    assert not velocity_zeros
    assert velocity_holds == [True]


def bind_hover_methods(node):
    for name in ("local_velocity_reason", "local_hover_reason", "local_hover_stable", "hover_progress"):
        setattr(node, name, getattr(Milestone3DemoSequenceNode, name).__get__(node))


def hover_node(height=2.0, origin=0.0):
    node = SimpleNamespace(
        now_s=lambda: 100.0, pose_fresh=lambda: True,
        last_local_velocity_time_s=100.0, latest_local_velocity=TwistStamped(),
        current_altitude_m=lambda: origin + height,
        takeoff_target_altitude_m=lambda: origin + 2.0,
        altitude_tolerance_m=0.1, stable_hover_max_horizontal_speed_mps=0.25,
        stable_hover_max_vertical_speed_mps=0.15,
        stable_hover_since_s=None, stage_hover_duration_s=2.0,
    )
    bind_hover_methods(node)
    return node


@pytest.mark.parametrize("origin", [0.0, 0.73, -2.5])
@pytest.mark.parametrize("height,accepted", [(1.899, False), (1.9, True), (1.95, True),
                                           (2.0, True), (2.1, True), (2.101, False)])
def test_hover_band_includes_boundaries_above_takeoff_origin(origin, height, accepted):
    node = hover_node(height, origin)
    assert node.local_hover_stable() is accepted
    assert Milestone3DemoSequenceNode.takeoff_altitude_reached(node) is (height >= 1.9)


def test_hover_reports_stale_nonfinite_and_fast_velocity():
    node = hover_node()
    node.last_local_velocity_time_s = 99.49
    assert "stale or missing" in node.hover_progress()
    node.last_local_velocity_time_s = 100.0
    node.latest_local_velocity.twist.linear.x = math.nan
    assert "non-finite" in node.local_hover_reason()
    node.latest_local_velocity.twist.linear.x = 0.26
    assert "horizontal speed" in node.local_hover_reason()
    node.latest_local_velocity.twist.linear.x = 0.0
    node.latest_local_velocity.twist.linear.z = -0.16
    assert "vertical speed" in node.local_hover_reason()
    assert "velocity_age=" in node.hover_progress()


@pytest.mark.parametrize("state", ["LOCAL_HANDOFF", "STAGE_HOVER"])
@pytest.mark.parametrize("height,direction", [(1.85, 1), (1.93, 1), (2.0, 0), (2.15, -1)])
def test_hover_actively_corrects_altitude_and_holds_takeoff_xy(state, height, direction):
    commands = []
    node = hover_node(height)
    node.__dict__.update(
        local_odom_mode=True, demo_state=state,
        takeoff_origin_xy=(0.0, 0.0), takeoff_origin_yaw_rad=0.0,
        current_xy=lambda: (0.5, 0.0), current_yaw_rad=lambda: 0.0,
        return_position_tolerance_m=0.2, yaw_deadband_rad=0.08,
        kp_xy=0.45, kp_z=0.3, kp_yaw=0.8,
        max_vel_xy_mps=0.6, max_vel_z_mps=0.3, max_yaw_rate_rps=0.4,
        clamp_command_to_perimeter_altitude=lambda command: command,
        command_projection_violation_reason=lambda command: None,
        publish_follow_command=commands.append,
    )
    Milestone3DemoSequenceNode.publish_takeoff_hold_command(node)
    assert len(commands) == 1
    assert commands[0].vx < 0.0
    assert (0 if commands[0].vz == 0 else math.copysign(1, commands[0].vz)) == direction


def test_hover_timeout_includes_the_actual_blocker():
    aborts = []
    node = hover_node(1.85)
    node.__dict__.update(
        demo_state="STAGE_HOVER", local_odom_mode=True,
        TERMINAL_STATES={"IDLE", "COMPLETE", "ABORT"},
        ACTIVE_ENGAGEMENT_STATES={"SEARCH", "FOLLOW", "DWELL"},
        RETURN_STATES={"RETURN_TO_START", "RETURN_HOME_HOLD"},
        local_health_reason=lambda: "", state_fresh=lambda: True,
        latest_state=SimpleNamespace(connected=True, armed=True, mode="OFFBOARD"),
        connection_lost_since_s=None, connection_loss_warned=False,
        perimeter_runtime_violation_reason=lambda: None, require_companion_active=False,
        mode_future=None, offboard_mode="OFFBOARD", mode_matches=lambda mode: True,
        stage_timeout_limit_s=lambda: 30.0, stage_elapsed_s=lambda: 30.1,
        enter_abort=aborts.append,
    )
    Milestone3DemoSequenceNode.run_safety_checks(node)
    assert len(aborts) == 1
    assert "stage timeout in STAGE_HOVER: altitude error" in aborts[0]
    assert "dwell=0.00/2.00s" in aborts[0]


def test_fresh_velocity_is_required_for_preflight_dwell():
    node = hover_node()
    node.__dict__.update(
        local_odom_mode=True, demo_state="IDLE", TERMINAL_STATES={"IDLE"},
        local_preflight_ready_since_s=98.0,
        latest_state=SimpleNamespace(connected=True, armed=False), estop_latched=False,
        last_perception_time_s=100.0, require_detector_target_on_start=False,
        state_fresh=lambda: True, local_health_reason=lambda: "",
    )
    node.last_local_velocity_time_s = 99.49
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s is None
    node.last_local_velocity_time_s = 100.0
    Milestone3DemoSequenceNode.update_local_preflight_readiness(node)
    assert node.local_preflight_ready_since_s == 100.0


def fake_vision_hold_node():
    now = [100.0]
    states, aborts, zeros = [], [], []
    node = SimpleNamespace(
        now_s=lambda: now[0], state_fresh=lambda: True,
        latest_state=SimpleNamespace(connected=True, armed=True),
        mode_matches=lambda mode: True, offboard_mode="OFFBOARD",
        pose_fresh=lambda: True, local_pose_discontinuous=False,
        vision_profile_ok=True, last_estimator_time_s=100.0,
        estimator_timeout_s=2.5, estimator_ok=True, estop_latched=False,
        perimeter_runtime_violation_reason=lambda: None,
        current_altitude_m=lambda: 2.0, takeoff_origin_altitude_m=0.0,
        min_follow_height_m=1.0, max_follow_height_m=2.5,
        last_zed_status_s=100.0, zed_status="ZED pose jump",
        accepted_vision_pose_count=10, vision_hold_start_count=10,
        vision_hold_resume_state="SEARCH",
        vision_hold_started_s=100.0, last_vision_time_s=100.0,
        set_demo_state=lambda state, reason="": states.append(state),
        enter_abort=aborts.append, publish_zero_command=lambda: zeros.append(True),
    )
    return node, now, states, aborts, zeros


def test_vision_hold_recovers_only_after_two_new_poses():
    node, now, states, aborts, zeros = fake_vision_hold_node()
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert zeros and not states and not aborts
    now[0] = 100.2
    node.zed_status = "OK"
    node.last_zed_status_s = 100.2
    node.accepted_vision_pose_count = 11
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert not states
    node.accepted_vision_pose_count = 12
    node.last_vision_time_s = 100.2
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert states == ["SEARCH"]


def test_stage_hover_vision_hold_restarts_stability_dwell():
    node, now, states, aborts, zeros = fake_vision_hold_node()
    node.demo_state = "STAGE_HOVER"
    node.tracks = {1: object()}
    node.clear_active_target = lambda: None
    node.set_demo_state = lambda state, reason="": (states.append(state),
                                                      setattr(node, "demo_state", state))
    Milestone3DemoSequenceNode.enter_vision_hold(node)
    assert node.tracks == {}
    assert node.vision_hold_resume_state == "STAGE_HOVER"
    node.zed_status = "OK"
    node.accepted_vision_pose_count = 12
    now[0] = 100.2
    node.last_zed_status_s = 100.2
    node.last_vision_time_s = 100.2
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert states == ["VISION_HOLD", "STAGE_HOVER"]
    assert not aborts


def test_persistent_vision_jump_or_invalid_px4_aborts():
    node, now, states, aborts, _ = fake_vision_hold_node()
    now[0] = 100.5
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert "persisted" in aborts[0]
    node, now, states, aborts, _ = fake_vision_hold_node()
    node.estimator_ok = False
    now[0] = 100.1
    Milestone3DemoSequenceNode.service_vision_hold(node)
    assert "unsafe" in aborts[0]


def test_search_with_no_target_keeps_zero_velocity_without_timeout():
    zeros, states = [], []
    node = SimpleNamespace(
        now_s=lambda: 1000.0, demo_state="SEARCH", local_odom_mode=True,
        ACTIVE_ENGAGEMENT_STATES={"SEARCH", "FOLLOW", "DWELL"},
        control_block_reason=lambda: "", select_target=lambda now: None,
        publish_zero_command=lambda: zeros.append(True),
        set_demo_state=lambda state, reason="": states.append(state),
        last_blocked_reason="",
    )
    assert Milestone3DemoSequenceNode.stage_timeout_limit_s(SimpleNamespace(
        demo_state="SEARCH", TERMINAL_STATES={"IDLE", "ABORT", "COMPLETE"})) is None
    for _ in range(3):
        Milestone3DemoSequenceNode.step_state_machine(node)
    assert len(zeros) == 3
    assert states == ["SEARCH"] * 3


def test_jump_enters_hold_but_invalid_px4_odometry_aborts_immediately():
    held, aborted = [], []
    node = SimpleNamespace(
        demo_state="FOLLOW", TERMINAL_STATES={"IDLE", "COMPLETE", "ABORT"},
        local_odom_mode=True,
        ACTIVE_ENGAGEMENT_STATES={"SEARCH", "FOLLOW", "DWELL"},
        RETURN_STATES={"RETURN_TO_START", "RETURN_HOME_HOLD"},
        local_health_reason=lambda: "ZED tracking unavailable: ZED pose jump",
        latest_state=SimpleNamespace(connected=True, armed=True),
        pose_fresh=lambda: True, local_pose_discontinuous=False,
        estimator_ok=True, last_estimator_time_s=100.0,
        estimator_timeout_s=2.5, now_s=lambda: 100.0,
        enter_vision_hold=lambda: held.append(True), enter_abort=aborted.append,
    )
    Milestone3DemoSequenceNode.run_safety_checks(node)
    assert held and not aborted
    node.estimator_ok = False
    Milestone3DemoSequenceNode.run_safety_checks(node)
    assert len(held) == 1
    assert aborted == ["ZED tracking unavailable: ZED pose jump"]


def test_single_rejected_zed_frame_enters_hold_before_next_timer_tick():
    held, aborted = [], []
    node = SimpleNamespace(
        now_s=lambda: 100.0, local_odom_mode=True, demo_state="FOLLOW",
        ACTIVE_ENGAGEMENT_STATES={"SEARCH", "FOLLOW", "DWELL"},
        RETURN_STATES={"RETURN_TO_START", "RETURN_HOME_HOLD"},
        latest_state=SimpleNamespace(connected=True, armed=True),
        pose_fresh=lambda: True, local_pose_discontinuous=False,
        estimator_ok=True, last_estimator_time_s=100.0,
        estimator_timeout_s=2.5, enter_vision_hold=lambda: held.append(True),
        enter_abort=aborted.append,
    )
    Milestone3DemoSequenceNode.zed_status_callback(node, String(data="ZED pose jump"))
    Milestone3DemoSequenceNode.zed_status_callback(node, String(data="OK"))
    assert held == [True] and not aborted
