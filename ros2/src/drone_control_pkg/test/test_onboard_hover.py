"""Flight logic tests use simulated telemetry; no FCU or ROS graph is required."""

from dataclasses import replace
import math

import pytest

from drone_control_pkg.onboard_hover_logic import HoverController, HoverSettings


def healthy(controller, now, pose=None):
    t = controller.telemetry
    t.connected = True
    t.estimator_ok = t.flow_ok = t.range_ok = True
    t.estimator_reason = "valid"
    t.landed = 1 if not t.armed else 2
    t.mode = t.mode or "POSCTL"
    position = pose or t.pose or (123.0, -456.0, 17.5, 1.2)
    controller.update_pose(position, "map", now)
    t.ground_distance = 0.2 + position[2] - 17.5
    for name in ("state", "extended", "estimator", "pose", "flow", "range"):
        t.received[name] = now


def ready(settings=None):
    controller = HoverController(settings or HoverSettings(enable_flight=True))
    for i in range(51):
        now = i * 0.05
        healthy(controller, now)
        controller.tick(now)
    return controller, now


def advance(controller, now, duration, *, follow=False, ground=False, ack=True):
    outputs = []
    for _ in range(round(duration / 0.05)):
        now += 0.05
        pose = controller.telemetry.pose
        if follow and controller.target_z is not None:
            pose = (*pose[:2], controller.target_z, pose[3])
        healthy(controller, now, pose)
        if ground:
            controller.telemetry.landed = 1
        output = controller.tick(now)
        outputs.append(output)
        if output.command and ack:
            kind, value = output.command
            controller.command_result(True, now)
            if kind == "mode":
                controller.telemetry.mode = value
            else:
                controller.telemetry.armed = value
    return now, outputs


def taking_off():
    c, now = ready()
    assert c.start(now)[0]
    now, outputs = advance(c, now, 2.0)
    assert c.state == "TAKEOFF"
    assert any(o.command == ("mode", "OFFBOARD") for o in outputs)
    assert any(o.command == ("arm", True) for o in outputs)
    return c, now


def test_diagnostics_never_commands_even_with_start_and_abort():
    c, now = ready(HoverSettings())
    assert not c.readiness(now)
    assert not c.start(now)[0]
    c.abort(now, "test")
    now, outputs = advance(c, now, 5.0)
    assert all(o.command is None and o.setpoint is None for o in outputs)


def test_hover_uses_nonzero_start_and_requires_altitude_dwell_then_lands():
    c, now = taking_off()
    assert c.origin == (123.0, -456.0, 17.5, 1.2)
    now, outputs = advance(c, now, 5.0, follow=True)
    assert c.state == "HOVER"
    assert outputs[-1].setpoint == pytest.approx((123.0, -456.0, 18.5, 1.2))
    assert all(o.setpoint is None or o.setpoint[:2] == c.origin[:2] for o in outputs)
    now, outputs = advance(c, now, 21.0, follow=True)
    assert c.state == "LANDING"
    assert any(o.command == ("mode", "AUTO.LAND") for o in outputs)
    assert not any(o.command == ("arm", False) for o in outputs)
    now, outputs = advance(c, now, 1.5, ground=True)
    assert any(o.command == ("arm", False) for o in outputs)
    assert c.state == "COMPLETE"


@pytest.mark.parametrize("field", ["estimator_ok", "flow_ok", "range_ok"])
def test_invalid_sensors_or_estimator_reset_readiness_dwell(field):
    c, now = ready()
    setattr(c.telemetry, field, False)
    assert not c.start(now)[0]
    setattr(c.telemetry, field, True)
    assert c.readiness(now)
    now, _ = advance(c, now, 1.0)
    assert not c.start(now)[0]
    now, _ = advance(c, now, 1.1)
    assert c.start(now)[0]


@pytest.mark.parametrize("topic", ["pose", "flow", "range", "state", "extended", "estimator"])
def test_missing_topic_blocks_start(topic):
    c, now = ready()
    del c.telemetry.received[topic]
    assert not c.start(now)[0]
    assert any(topic in reason for reason in c.readiness(now))


def test_stale_pose_aborts_without_setpoint():
    c, now = taking_off()
    c.telemetry.received["pose"] = now - 0.6
    output = c.tick(now)
    assert c.state == "LANDING"
    assert output.setpoint is None
    assert output.command == ("mode", "AUTO.LAND")


def test_prearm_fault_stops_without_changing_disarmed_fcu_to_auto_land():
    c, now = ready()
    assert c.start(now)[0]
    c.telemetry.estimator_ok = False
    c.telemetry.estimator_reason = "horizontal velocity invalid"
    output = c.tick(now)
    assert c.state == "ABORT"
    assert output.command is None and output.setpoint is None
    assert c.telemetry.mode == "POSCTL"


def test_abort_and_operator_takeover_require_fresh_node_before_restart():
    for state in ("ABORT", "OPERATOR_CONTROL", "COMPLETE", "FAILSAFE"):
        c, now = ready()
        c.transition(state, now, "previous run ended")
        accepted, reason = c.start(now)
        assert not accepted
        assert f"restart node after {state.lower()} run" in reason


@pytest.mark.parametrize("pose", [None, (math.nan, 0.0, 0.0, 0.0)])
def test_invalid_pose_in_flight_is_handled(pose):
    c, now = taking_off()
    c.telemetry.pose = pose
    output = c.tick(now)
    assert c.state == "LANDING"
    assert output.setpoint is None


def test_pose_jump_does_not_reanchor_or_command_old_position():
    c, now = taking_off()
    origin = c.origin
    c.update_pose((1000.0, 0.0, 0.0, 0.0), "map", now + 0.05)
    output = c.tick(now + 0.05)
    assert c.state == "LANDING"
    assert c.origin == origin
    assert output.setpoint is None


def test_frame_change_invalidates_reference():
    c, now = taking_off()
    c.update_pose(c.telemetry.pose, "different_frame", now + 0.05)
    assert c.state == "LANDING"


def test_reconnect_clears_freshness_and_does_not_resume_flight():
    c, now = taking_off()
    c.connection_reset(now)
    assert c.state == "LANDING"
    assert not c.telemetry.received
    assert c.tick(now).setpoint is None
    assert not c.start(now)[0]


def test_no_climb_times_out_instead_of_accepting_hover():
    c, now = taking_off()
    now, _ = advance(c, now, 6.0)
    assert c.state == "LANDING"
    assert c.target_z <= c.origin[2] + 0.25
    assert "no range-confirmed liftoff" in c.reason


def test_false_local_climb_with_flat_range_aborts_before_reported_overshoot():
    c, now = taking_off()
    for i in range(90):
        now += 0.05
        height = min(0.6, 0.6 * (i + 1) / 90)
        healthy(c, now, (123.0, -456.0, 17.5 + height, 1.2))
        c.telemetry.ground_distance = c.ground_distance_start
        output = c.tick(now)
        if c.state == "LANDING":
            break
    assert c.state == "LANDING"
    assert "local altitude disagrees with downward range" in c.reason
    assert height < c.settings.takeoff_altitude_m + c.settings.altitude_overshoot_m
    assert output.setpoint is None
    assert output.command == ("mode", "AUTO.LAND")


def test_false_local_altitude_cannot_confirm_hover_without_range_climb():
    c, now = taking_off()
    for i in range(50):
        now += 0.05
        height = (i + 1) * 0.02
        healthy(c, now, (123.0, -456.0, 17.5 + height, 1.2))
        c.telemetry.ground_distance = c.ground_distance_start + max(0.0, height - 0.2)
        c.tick(now)
    for _ in range(30):
        now += 0.05
        healthy(c, now, (123.0, -456.0, 18.5, 1.2))
        c.telemetry.ground_distance = c.ground_distance_start + 0.8
        c.tick(now)
    assert c.state == "TAKEOFF"
    assert not c.aborted


def test_horizontal_excursion_and_altitude_overshoot_abort():
    for delta in [(0.51, 0, 0), (0, 0, 1.31)]:
        c, now = taking_off()
        p = c.telemetry.pose
        c.telemetry.pose = (*(p[i] + delta[i] for i in range(3)), p[3])
        output = c.tick(now)
        assert c.state == "LANDING"
        assert output.setpoint is None


def test_operator_mode_change_stops_output_and_does_not_reengage():
    c, now = taking_off()
    c.telemetry.mode = "POSCTL"
    output = c.tick(now)
    assert c.state == "OPERATOR_CONTROL"
    assert output.setpoint is None and output.command is None
    now, outputs = advance(c, now, 10.0)
    assert not any(o.command or o.setpoint for o in outputs)


def test_operator_takeover_wins_if_estimator_fails_in_same_cycle():
    c, now = taking_off()
    c.telemetry.mode = "POSCTL"
    c.telemetry.estimator_ok = False
    c.telemetry.estimator_reason = "horizontal velocity invalid"
    output = c.tick(now)
    assert c.state == "OPERATOR_CONTROL"
    assert output.setpoint is None and output.command is None


def test_rejected_arming_aborts():
    c, now = ready()
    c.start(now)
    while c.pending != ("arm", True):
        now += 0.05
        healthy(c, now)
        output = c.tick(now)
        if output.command == ("mode", "OFFBOARD"):
            c.command_result(True, now)
            c.telemetry.mode = "OFFBOARD"
    c.command_result(False, now, "denied")
    assert c.state == "LANDING"


def test_command_timeout_stops_stream_and_never_retries_ambiguous_arm():
    c, now = taking_off()
    c.pending = ("arm", True)
    c.pending_since = now - 3.0
    output = c.tick(now)
    assert c.state == "FAILSAFE"
    assert output.setpoint is None and output.command is None
    assert not c.start(now)[0]


def test_mode_rejection_retries_are_bounded():
    c, now = ready()
    c.start(now)
    mode_requests = 0
    for _ in range(130):
        now += 0.05
        healthy(c, now)
        output = c.tick(now)
        if output.command:
            if output.command == ("mode", "OFFBOARD"):
                mode_requests += 1
            c.command_result(False, now, "denied")
    assert mode_requests == 3
    assert c.state in {"LANDING", "ABORT", "FAILSAFE"}


def test_stale_landed_state_never_disarms():
    c, now = taking_off()
    c.abort(now, "test")
    c.telemetry.mode = "AUTO.LAND"
    c.telemetry.landed = 1
    c.telemetry.received["extended"] = now - 3.0
    for _ in range(40):
        now += 0.05
        c.telemetry.received["state"] = now
        assert c.tick(now).command is None
    assert c.state == "LANDING"


def test_repeated_abort_preserves_original_reason_and_landing_deadline():
    c, now = taking_off()
    c.abort(now, "initial climb height limit exceeded")
    entered = c.entered
    c.abort(now + 5, "operator abort")
    assert c.state == "LANDING"
    assert c.reason == "initial climb height limit exceeded"
    assert c.entered == entered

    c.transition("LANDING", now, "hover complete")
    c.aborted = False
    c.abort(now + 5, "operator abort")
    assert c.aborted
    assert c.reason == "operator abort"
    assert c.entered == entered


def test_invalid_settings_fail_before_start():
    for values in ({"takeoff_altitude_m": math.nan}, {"publish_rate_hz": 2.0},
                   {"takeoff_rate_m_s": -1.0}, {"range_min_m": 100.0}):
        with pytest.raises(ValueError):
            replace(HoverSettings(), **values)


def test_hover_altitude_loss_cannot_be_reported_as_success():
    c, now = taking_off()
    now, _ = advance(c, now, 5.0, follow=True)
    assert c.state == "HOVER"
    p = c.telemetry.pose
    c.telemetry.pose = (*p[:2], p[2] - 0.31, p[3])
    output = c.tick(now)
    assert c.state == "LANDING"
    assert output.setpoint is None
    assert "altitude left hover band" in c.reason
