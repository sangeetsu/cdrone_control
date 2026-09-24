"""Exercise unaided climb, controller dynamics and one-way PX4 handover offline."""

from dataclasses import replace
import math

import pytest

from drone_control_pkg.onboard_hover_logic import HoverController, HoverSettings


class Flight:
    """Independent vertical point-mass plant, with ground contact and motor lag."""
    def __init__(self, *, flight=True, hover=0.62):
        self.c = HoverController(HoverSettings(
            enable_flight=flight, initial_climb_enabled=True,
            initial_climb_hover_thrust=hover))
        self.now = 100.0
        self.height = self.speed = self.motor_thrust = 0.0
        self.aiding = False
        self.outputs = []
        self.feed()
        self.run(2.1)

    def feed(self):
        t = self.c.telemetry
        t.connected, t.mode = True, t.mode or "POSCTL"
        t.vertical_estimator_ok = t.flow_ok = t.range_ok = True
        t.vertical_estimator_reason = "valid"
        t.estimator_ok = self.aiding
        t.estimator_reason = "valid" if self.aiding else "horizontal aiding inactive"
        t.attitude = (0.0, 0.0, 0.7)
        t.ground_distance = 0.075 + self.height
        t.landed = 2 if self.height > 0.03 else 1
        # Deliberately arbitrary local frame: relative range defines physical climb.
        self.c.update_pose((12.0, -9.0, 17.5 + self.height, 0.7), "map", self.now)
        for key in ("state", "extended", "estimator", "imu", "pose", "flow", "range"):
            t.received[key] = self.now

    def step(self, *, plant=False, echo=True):
        dt = 0.02
        self.now += dt
        self.feed()
        if echo and self.c.state == "POSITION_HANDOVER":
            self.c.telemetry.position_target = (*self.c.origin[:2], self.c.target_z, self.c.origin[3])
            self.c.telemetry.received["position_target"] = self.now
        output = self.c.tick(self.now)
        self.outputs.append(output)
        assert not (output.setpoint is not None and output.attitude_setpoint is not None)
        if output.command is not None:
            kind, value = output.command
            self.c.command_result(True, self.now)
            if kind == "arm":
                self.c.telemetry.armed = value
            else:
                self.c.telemetry.mode = value
        if plant:
            desired = output.attitude_setpoint[1] if output.attitude_setpoint else 0.0
            self.motor_thrust += dt / (0.12 + dt) * (desired - self.motor_thrust)
            acceleration = 9.80665 * (self.motor_thrust / 0.62 - 1) - 0.25 * self.speed
            self.speed += acceleration * dt
            self.height += self.speed * dt
            if self.height < 0:
                self.height = self.speed = 0.0
        return output

    def run(self, seconds, **kwargs):
        for _ in range(round(seconds / 0.02)):
            self.step(**kwargs)

    def start(self):
        assert self.c.start(self.now)[0]
        while self.c.state != "INITIAL_CLIMB":
            self.step()


def test_initial_climb_requires_explicit_hover_thrust_and_vertical_health():
    f = Flight()
    f.c.settings = replace(f.c.settings, initial_climb_hover_thrust=0.0)
    assert "hover thrust" in f.c.start(f.now)[1]
    f.c.settings = replace(f.c.settings, initial_climb_hover_thrust=0.62)
    f.c.telemetry.vertical_estimator_ok = False
    assert not f.c.start(f.now)[0]


def test_unaided_warmup_and_arm_only_send_zero_thrust_attitude():
    f = Flight()
    f.start()
    attitude = [o.attitude_setpoint for o in f.outputs if o.attitude_setpoint is not None]
    assert attitude and all(yaw == 0.7 and thrust == 0 for yaw, thrust in attitude)
    assert all(o.setpoint is None for o in f.outputs)
    assert any(o.command == ("arm", True) for o in f.outputs)


def test_initial_diagnostics_never_publish_or_command():
    f = Flight(flight=False)
    assert not f.c.start(f.now)[0]
    f.run(5)
    assert all(o.setpoint is None and o.attitude_setpoint is None and o.command is None
               for o in f.outputs)


def test_closed_loop_climbs_and_hands_over_after_stable_aiding():
    f = Flight()
    f.start()
    initial_time = f.now
    seen_handover = False
    previous_thrust = 0.0
    while f.now - initial_time < 9:
        f.aiding = f.height > 0.12
        output = f.step(plant=True)
        if output.attitude_setpoint:
            thrust = output.attitude_setpoint[1]
            assert 0 <= thrust <= 0.75
            assert thrust - previous_thrust <= 0.25 * 0.02 + 1e-9
            previous_thrust = thrust
        seen_handover |= f.c.state == "POSITION_HANDOVER"
        if f.c.state == "TAKEOFF":
            break
        assert f.c.state == "INITIAL_CLIMB" or f.c.state == "POSITION_HANDOVER", f.c.reason
    assert seen_handover
    assert f.c.state == "TAKEOFF"
    assert 0.20 < f.height < 0.4
    assert f.c.origin[2] == pytest.approx(17.5, abs=0.03)
    assert output.attitude_setpoint is None
    # Aiding loss after handover must land, never revert to the unaided climb.
    f.aiding = False
    output = f.step()
    assert f.c.state == "LANDING"
    assert output.command == ("mode", "AUTO.LAND")
    assert output.attitude_setpoint is None and output.setpoint is None


def test_spinning_motors_or_local_z_drift_do_not_count_as_liftoff():
    f = Flight()
    f.start()
    for _ in range(275):
        f.now += 0.02
        f.feed()
        f.c.telemetry.landed = 2  # PX4 falsely reports takeoff, as in the real log.
        f.c.telemetry.pose = (12.0, -9.0, 18.2, 0.7)
        output = f.c.tick(f.now)
        if f.c.state == "LANDING":
            break
    assert f.c.state == "LANDING"
    assert "no measured liftoff" in f.c.reason
    assert output.attitude_setpoint is None
    assert output.command == ("mode", "AUTO.LAND")


def test_missing_aiding_times_out_even_when_height_is_reached():
    f = Flight()
    f.start()
    f.height = 0.25
    f.run(8.1)
    assert f.c.state == "LANDING"
    assert "stable position aiding" in f.c.reason


def test_aiding_dwell_resets_on_dropout_and_position_reset():
    f = Flight()
    f.start()
    f.height = 0.25
    f.run(1.0)
    f.aiding = True
    f.run(0.6)
    assert f.c.climb.aiding_since is not None
    f.aiding = False
    f.step()
    assert f.c.climb.aiding_since is None
    f.aiding = True
    f.run(0.6)
    f.c.update_pose((100.0, -20.0, 17.75, 0.7), "map", f.now)
    assert f.c.climb.aiding_since is None
    assert f.c.state == "INITIAL_CLIMB"


def test_handover_requires_new_matching_px4_position_target():
    f = Flight()
    f.start()
    f.height = 0.25
    f.aiding = True
    f.c.telemetry.position_target = (12.0, -9.0, 17.75, 0.7)
    f.c.telemetry.received["position_target"] = f.now
    while f.c.state == "INITIAL_CLIMB":
        f.step(echo=False)
    assert f.c.state == "POSITION_HANDOVER"
    f.c.telemetry.position_target = (1234.0, -9.0, 17.75, 0.7)
    f.c.telemetry.received["position_target"] = f.now + 0.01
    f.run(2.1, echo=False)
    assert f.c.state == "LANDING"
    assert "handover not confirmed" in f.c.reason


@pytest.mark.parametrize("fault", ["stale_range", "bad_range", "stale_imu", "bad_attitude",
                                  "tilt", "vertical_estimator", "height", "yaw", "timing"])
def test_initial_climb_faults_stop_attitude_output_and_request_landing(fault):
    f = Flight()
    f.start()
    f.run(0.5)
    f.now += 0.02
    f.feed()
    t = f.c.telemetry
    if fault == "stale_range":
        t.received["range"] -= 1
    elif fault == "bad_range":
        t.range_ok, t.ground_distance = False, None
    elif fault == "stale_imu":
        t.received["imu"] -= 1
    elif fault == "bad_attitude":
        t.attitude = None
    elif fault == "tilt":
        t.attitude = (0.3, 0.0, 0.7)
    elif fault == "vertical_estimator":
        t.vertical_estimator_ok = False
    elif fault == "height":
        t.ground_distance = 0.6
    elif fault == "yaw":
        t.attitude = (0, 0, 1.5)
    elif fault == "timing":
        f.now += 0.7
    output = f.c.tick(f.now)
    assert f.c.state == "LANDING"
    assert output.attitude_setpoint is None and output.setpoint is None


def test_operator_mode_change_and_disarm_never_resume_thrust():
    for change in ("mode", "disarm"):
        f = Flight()
        f.start()
        f.run(0.5)
        if change == "mode":
            f.c.telemetry.mode = "ALTCTL"
        else:
            f.c.telemetry.armed = False
        output = f.c.tick(f.now)
        assert f.c.state in {"OPERATOR_CONTROL", "ABORT"}
        assert output.attitude_setpoint is None and output.command is None


def test_initial_climb_frame_change_or_reconnect_aborts():
    for fault in ("frame", "connection"):
        f = Flight()
        f.start()
        if fault == "frame":
            f.c.update_pose((12.0, -9.0, 17.5, 0.7), "other_map", f.now)
        else:
            f.c.connection_reset(f.now)
        assert f.c.state == "LANDING"
        assert f.c.tick(f.now).attitude_setpoint is None


def test_single_healthy_estimator_sample_does_not_satisfy_handover_dwell():
    f = Flight()
    f.start()
    f.height = 0.25
    f.run(1.0)
    f.aiding = True
    estimator_time = f.now
    for _ in range(65):
        f.now += 0.02
        f.feed()
        f.c.telemetry.received["estimator"] = estimator_time
        f.c.tick(f.now)
    assert f.c.state == "INITIAL_CLIMB"
    assert f.c.climb.aiding_since is None


@pytest.mark.parametrize("values", [
    {"initial_climb_hover_thrust": math.nan}, {"initial_climb_hover_thrust": 0.8},
    {"initial_climb_max_thrust": 1.1}, {"initial_climb_height_m": 0.1},
    {"initial_climb_min_range_m": 0.05}, {"initial_climb_timeout_s": 30.0},
    {"initial_climb_max_tilt_rad": 0.6}, {"initial_climb_rate_m_s": 1.0},
])
def test_unsafe_initial_climb_settings_rejected(values):
    with pytest.raises(ValueError):
        HoverSettings(initial_climb_enabled=True, **values)
