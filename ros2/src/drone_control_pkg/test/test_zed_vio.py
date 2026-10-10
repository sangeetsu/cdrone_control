"""ZED mount math, freshness gate, and ZED-only hover behavior."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from drone_control_pkg.onboard_hover_logic import HoverController, HoverSettings
from drone_control_pkg.zed_vio import Mount, PoseGate, load_mount, transform_camera_to_body


def test_mount_requires_measured_values(tmp_path):
    path = tmp_path / "mount.yaml"
    path.write_text("calibrated: false\nbody_to_camera_translation_m: null\n")
    assert load_mount(path) is None
    path.write_text("calibrated: true\nbody_to_camera_translation_m: [0.2, 0, 0]\n"
                    "body_to_camera_rpy_rad: [0, 0, 3.141592653589793]\n")
    assert load_mount(path).body_to_camera_translation_m == (0.2, 0, 0)
    path.write_text("calibrated: true\nbody_to_camera_translation_m: [0, nan, 0]\n"
                    "body_to_camera_rpy_rad: [0, 0, 0]\n")
    with pytest.raises(ValueError):
        load_mount(path)


def test_camera_pose_and_covariance_transform_include_lever_arm():
    mount = Mount((0.2, 0, 0), (0, 0, math.pi))
    camera_q = Rotation.from_euler("z", math.pi).as_quat()
    covariance = np.diag([0.01, 0.01, 0.01, 0.04, 0.04, 0.04])
    position, quaternion, body_covariance = transform_camera_to_body(
        (2, 0, 0), camera_q, covariance, mount)
    assert position == pytest.approx((1.8, 0, 0))
    assert Rotation.from_quat(quaternion).magnitude() == pytest.approx(0, abs=1e-12)
    assert body_covariance[1, 1] == pytest.approx(0.01 + 0.2**2 * 0.04)
    assert body_covariance[2, 2] == pytest.approx(0.01 + 0.2**2 * 0.04)


def test_pose_gate_rejects_stale_duplicate_lost_and_jumped_frames():
    gate = PoseGate()
    p, q = np.zeros(3), np.array((0, 0, 0, 1))
    assert gate.accept("OK", 1_000_000_000, 1_050_000_000, p, q)[0]
    assert not gate.accept("OK", 1_000_000_000, 1_050_000_000, p, q)[0]
    assert not gate.accept("OK", 1_100_000_000, 1_500_000_000, p, q)[0]
    assert not gate.accept("UNAVAILABLE", 1_100_000_000, 1_120_000_000, p, q)[0]
    assert not gate.accept("OK", 1_100_000_000, 1_120_000_000, np.array((1, 0, 0)), q)[0]
    assert gate.accept("OK", 1_100_000_000, 1_120_000_000, np.array((0.1, 0, 0)), q)[0]


def feed_zed(controller, now, local_z=17.5, vision_z=3.0, predicted=True):
    telemetry = controller.telemetry
    telemetry.connected = True
    telemetry.mode = telemetry.mode or "POSCTL"
    telemetry.landed = 2 if telemetry.armed and vision_z > 3.03 else 1
    telemetry.estimator_ok = True
    telemetry.estimator_reason = "valid"
    telemetry.update_relative_position_prediction(predicted, now)
    telemetry.vision_profile_ok = True
    controller.update_pose((1.0, 2.0, local_z, 0.3), "map", now)
    controller.update_vision_pose((10.0, 20.0, vision_z, 0.3), "zed_odom", now)
    for name in ("state", "extended", "estimator"):
        telemetry.received[name] = now


def test_zed_hover_requires_vision_and_uses_it_for_liftoff_and_hover():
    controller = HoverController(HoverSettings(enable_flight=True, localization_source="zed"))
    now = 0.0
    for _ in range(51):
        now += 0.05
        feed_zed(controller, now)
        controller.tick(now)
    assert not controller.readiness(now)
    assert controller.start(now)[0]
    saw_handover = False
    for _ in range(170):
        now += 0.05
        height = min(1.0, max(0.0, (now - 4.0) * 0.3))
        feed_zed(controller, now, 17.5 + height, 3.0 + height)
        output = controller.tick(now)
        if output.command:
            kind, value = output.command
            controller.command_result(True, now)
            if kind == "mode":
                controller.telemetry.mode = value
            else:
                controller.telemetry.armed = value
        saw_handover |= controller.state == "POSITION_HANDOVER"
        if controller.state == "HOVER":
            break
    assert not saw_handover
    assert controller.state == "HOVER", controller.reason
    assert output.setpoint is not None
    controller.telemetry.received["vision"] = now - 0.6
    output = controller.tick(now)
    assert controller.state == "LANDING"
    assert output.setpoint is None
    assert output.command == ("mode", "AUTO.LAND")


def test_zed_hover_rejects_local_only_climb_and_flow_is_not_required():
    controller = HoverController(HoverSettings(enable_flight=True, localization_source="zed"))
    now = 0.0
    for _ in range(51):
        now += 0.05
        feed_zed(controller, now)
        controller.tick(now)
    assert "flow" not in ";".join(controller.readiness(now))
    assert "range" not in ";".join(controller.readiness(now))
    assert controller.start(now)[0]
    for _ in range(100):
        now += 0.05
        feed_zed(controller, now, local_z=17.5 + min(0.6, (now - 2.5) * 0.2))
        output = controller.tick(now)
        if output.command:
            kind, value = output.command
            controller.command_result(True, now)
            if kind == "mode":
                controller.telemetry.mode = value
            else:
                controller.telemetry.armed = value
        if controller.state == "LANDING":
            break
    assert controller.state == "LANDING"
    assert "ZED" in controller.reason


def test_zed_no_liftoff_reports_measured_and_requested_rise():
    controller = HoverController(HoverSettings(enable_flight=True, localization_source="zed"))
    now = 0.0
    for _ in range(51):
        now += 0.05
        feed_zed(controller, now)
        controller.tick(now)
    assert controller.start(now)[0]
    takeoff_entered = None
    previous_target = None
    for _ in range(260):
        now += 0.05
        feed_zed(controller, now)
        output = controller.tick(now)
        if controller.state == "TAKEOFF" and takeoff_entered is None:
            takeoff_entered = now
        if output.setpoint is not None:
            target = output.setpoint[2]
            assert target <= controller.telemetry.pose[2] + 0.6 + 1e-9
            if previous_target is not None:
                assert target - previous_target <= 0.3 * 0.05 + 1e-9
            previous_target = target
        if output.command:
            kind, value = output.command
            controller.command_result(True, now)
            if kind == "mode":
                controller.telemetry.mode = value
            else:
                controller.telemetry.armed = value
        if controller.state == "LANDING":
            break
    assert controller.state == "LANDING"
    assert takeoff_entered is not None and now - takeoff_entered >= 8.0
    assert "no ZED-confirmed liftoff" in controller.reason
    assert "ZED rise=0.000 m" in controller.reason
    assert "local rise=0.000 m" in controller.reason
    assert "requested rise=" in controller.reason
    assert "requested rise=0.600 m" in controller.reason


@pytest.mark.parametrize("values", [
    {"zed_takeoff_setpoint_lead_m": 0.7},
    {"zed_takeoff_no_lift_timeout_s": 5.0},
])
def test_zed_takeoff_bounds_reject_unsafe_overrides(values):
    with pytest.raises(ValueError):
        HoverSettings(localization_source="zed", **values)


def test_brief_px4_prediction_dip_does_not_abort_zed_warmup():
    controller = HoverController(HoverSettings(enable_flight=True, localization_source="zed"))
    now = 0.0
    for _ in range(51):
        now += 0.05
        feed_zed(controller, now)
        controller.tick(now)
    assert controller.start(now)[0]
    for _ in range(12):  # The observed at-rest dip lasted about 0.6 s.
        now += 0.05
        feed_zed(controller, now, predicted=False)
        output = controller.tick(now)
        assert controller.state == "WARMUP"
        assert output.command is None
    now += 0.05
    feed_zed(controller, now)
    controller.tick(now)
    assert controller.state == "WARMUP"
    assert not controller.aborted


def test_sustained_prediction_loss_aborts_before_arming_without_land_command():
    controller = HoverController(HoverSettings(enable_flight=True, localization_source="zed"))
    now = 0.0
    for _ in range(51):
        now += 0.05
        feed_zed(controller, now)
        controller.tick(now)
    assert controller.start(now)[0]
    for _ in range(22):
        now += 0.05
        feed_zed(controller, now, predicted=False)
        output = controller.tick(now)
        if controller.state == "ABORT":
            break
    assert controller.state == "ABORT"
    assert "relative-position prediction unavailable" in controller.reason
    assert output.command is None and output.setpoint is None
    assert not controller.telemetry.armed
