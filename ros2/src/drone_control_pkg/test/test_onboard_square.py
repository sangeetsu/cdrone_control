"""ZED square flight logic tests with simulated local and vision poses."""

import math

import pytest

from drone_control_pkg.onboard_hover_logic import HoverController, HoverSettings


START = (123.0, -456.0, 17.5, 1.2)


def feed(controller, now, pose=START):
    telemetry = controller.telemetry
    telemetry.connected = True
    telemetry.mode = telemetry.mode or "POSCTL"
    telemetry.landed = 2 if telemetry.armed else 1
    telemetry.estimator_ok = True
    telemetry.estimator_reason = "valid"
    telemetry.vision_profile_ok = True
    telemetry.relative_position_predicted = True
    controller.update_pose(pose, "map", now)
    controller.update_vision_pose(pose, "zed", now)
    for name in ("state", "extended", "estimator", "pose", "vision"):
        telemetry.received[name] = now


def ready(settings=None):
    controller = HoverController(settings or HoverSettings(
        localization_source="zed", mission_mode="square", enable_flight=True))
    for step in range(51):
        now = step * 0.05
        feed(controller, now)
        controller.tick(now)
    return controller, now


def step(controller, now, pose):
    now += 0.05
    feed(controller, now, pose)
    output = controller.tick(now)
    if output.command:
        kind, value = output.command
        controller.command_result(True, now)
        if kind == "mode":
            controller.telemetry.mode = value
        else:
            controller.telemetry.armed = value
    return now, output


def reach_square(controller, now):
    assert controller.start(now)[0]
    pose = START
    for _ in range(700):
        now, output = step(controller, now, pose)
        if output.setpoint:
            pose = output.setpoint
        if controller.state == "SQUARE":
            return now, pose
    pytest.fail("square did not start")


def reach_hover(controller, now):
    assert controller.start(now)[0]
    pose = START
    for _ in range(300):
        now, output = step(controller, now, pose)
        if output.setpoint:
            pose = output.setpoint
        if controller.state == "HOVER":
            return now, pose
    pytest.fail("hover did not start")


def test_heading_relative_vertices_have_four_one_meter_edges():
    for yaw in (0.0, math.pi / 2, 1.2):
        origin = (123.0, -456.0, 17.5, yaw)
        vertices = HoverController.square_path(origin, 1.0)
        assert vertices[0] == vertices[-1] == origin[:2]
        assert vertices[1] == pytest.approx((123 + math.cos(yaw), -456 + math.sin(yaw)))
        assert vertices[2] == pytest.approx((123 + math.cos(yaw) - math.sin(yaw),
                                             -456 + math.sin(yaw) + math.cos(yaw)))
        assert all(math.dist(a, b) == pytest.approx(1.0)
                   for a, b in zip(vertices, vertices[1:]))


def test_square_waits_for_measured_corners_and_returns_to_land():
    controller, now = ready()
    now, pose = reach_square(controller, now)
    assert controller.origin == START
    targets = []
    previous_xy = controller.origin[:2]
    seen_legs = {controller.square_leg}
    for _ in range(1300):
        now, output = step(controller, now, pose)
        if output.setpoint:
            target = output.setpoint
            assert target[2:] == pytest.approx((START[2] + 1.0, START[3]))
            assert math.dist(target[:2], previous_xy) <= 0.011
            assert math.dist(target[:2], pose[:2]) <= 0.301
            previous_xy = target[:2]
            pose = target
            targets.append(target)
        seen_legs.add(controller.square_leg)
        if controller.state == "LANDING":
            break
    assert seen_legs == {0, 1, 2, 3}
    assert controller.state == "LANDING"
    assert controller.reason == "square complete"
    assert math.dist(pose[:2], START[:2]) <= 0.15
    assert targets
    # PX4 reports touchdown and disarm only after AUTO.LAND takes over.
    for _ in range(50):
        now += 0.05
        feed(controller, now, pose)
        controller.telemetry.landed = 1
        output = controller.tick(now)
        if output.command:
            kind, value = output.command
            controller.command_result(True, now)
            if kind == "mode":
                controller.telemetry.mode = value
            else:
                controller.telemetry.armed = value
        if controller.state == "COMPLETE":
            break
    assert controller.state == "COMPLETE"


def test_square_stops_setpoint_at_lead_limit_then_times_out():
    controller, now = ready()
    now, pose = reach_square(controller, now)
    outputs = []
    for _ in range(420):
        now, output = step(controller, now, pose)
        outputs.append(output)
        if controller.state == "LANDING":
            break
    assert max(output.setpoint[0] for output in outputs if output.setpoint) > START[0]
    assert controller.state == "LANDING"
    assert "waypoint 1 timed out" in controller.reason
    assert outputs[-1].setpoint is None


def test_square_corner_requires_measured_arrival_and_one_second_dwell():
    controller, now = ready()
    now, pose = reach_square(controller, now)
    for _ in range(160):
        now, output = step(controller, now, pose)
        if output.setpoint:
            pose = output.setpoint
        if controller.square_progress_m >= 1.0:
            break
    assert controller.square_leg == 0
    assert controller.square_progress_m == pytest.approx(1.0)
    # The setpoint has arrived, but the measured pose still needs to settle.
    for _ in range(12):
        now, _ = step(controller, now, pose)
    assert controller.square_leg == 0
    for _ in range(12):
        now, _ = step(controller, now, pose)
    assert controller.square_leg == 1


def test_square_waits_for_start_position_then_aborts_if_it_never_settles():
    controller, now = ready()
    now, pose = reach_hover(controller, now)
    offset_pose = (pose[0] + 0.2, pose[1], pose[2], pose[3])
    for _ in range(820):
        now, output = step(controller, now, offset_pose)
        assert controller.state != "SQUARE"
        if controller.state == "LANDING":
            break
        assert output.setpoint[:2] == START[:2]
    assert controller.state == "LANDING"
    assert "square start did not settle" in controller.reason


@pytest.mark.parametrize("offset,expected", [((1.9, 0.0), "excursion"),
                                                   ((0.0, 0.4), "path error")])
def test_square_deviation_requests_landing(offset, expected):
    controller, now = ready(HoverSettings(
        localization_source="zed", mission_mode="square", enable_flight=True,
        max_pose_jump_m=3.0))
    now, pose = reach_square(controller, now)
    # Offsets are expressed along/perpendicular to the first edge.
    yaw = START[3]
    x = START[0] + offset[0] * math.cos(yaw) - offset[1] * math.sin(yaw)
    y = START[1] + offset[0] * math.sin(yaw) + offset[1] * math.cos(yaw)
    now, output = step(controller, now, (x, y, pose[2], pose[3]))
    assert controller.state == "LANDING"
    assert expected in controller.reason
    assert output.setpoint is None


def test_square_keeps_vision_health_and_operator_takeover_guards():
    controller, now = ready()
    now, pose = reach_square(controller, now)
    controller.telemetry.estimator_ok = False
    now += 0.05
    feed(controller, now, pose)
    controller.telemetry.estimator_ok = False
    output = controller.tick(now)
    assert controller.state == "LANDING"
    assert output.setpoint is None

    controller, now = ready()
    now, pose = reach_square(controller, now)
    controller.telemetry.mode = "POSCTL"
    now, output = step(controller, now, pose)
    assert controller.state == "OPERATOR_CONTROL"
    assert output.command is None and output.setpoint is None


def test_square_diagnostics_mode_never_commands_when_flight_disabled():
    controller, now = ready(HoverSettings(localization_source="zed", mission_mode="square"))
    assert not controller.start(now)[0]
    for _ in range(50):
        now, output = step(controller, now, START)
        assert output.command is None and output.setpoint is None


def test_square_requires_zed_and_sufficient_excursion_limit():
    with pytest.raises(ValueError, match="requires ZED"):
        HoverSettings(mission_mode="square")
    with pytest.raises(ValueError, match="fit inside excursion"):
        HoverSettings(localization_source="zed", mission_mode="square",
                      square_max_excursion_m=1.5)
