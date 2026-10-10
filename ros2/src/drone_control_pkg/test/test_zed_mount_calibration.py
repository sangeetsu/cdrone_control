"""Synthetic full-rank and degenerate ZED mount calibration checks."""

import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from drone_control_pkg.zed_mount_calibrate import _render_mount
from drone_control_pkg.zed_mount_calibration import (
    PivotCalibrationError, PoseSample, camera_orientation_in_body,
    fit_axis_pivots, mount_from_motion, stable_pose, validate_axis_motion,
)
from drone_control_pkg.zed_vio import load_mount


def synthetic_motion(axis_spans_deg=None):
    axis_spans_deg = axis_spans_deg or {"yaw": 57.3, "pitch": 57.3, "roll": 57.3}
    camera_in_body = Rotation.from_euler("xyz", (math.pi, 0.08, -0.15))
    body_start = Rotation.from_euler("xyz", (0.02, -0.03, 0.4))
    lever_body = np.array((0.21, -0.06, 0.12))
    center_start = np.array((2.0, -0.5, 0.8))

    def sample(center, body_rotation):
        return PoseSample(
            center + body_rotation.apply(lever_body),
            (body_rotation * camera_in_body).as_quat())

    start = sample(center_start, body_start)
    forward = sample(center_start + body_start.apply((0.5, 0, 0)), body_start)
    left = sample(center_start + body_start.apply((0.5, 0.45, 0)), body_start)

    def sweep(axis, seed):
        rng = np.random.default_rng(seed + 17)
        poses = []
        pivot_center = center_start + seed * np.array((0.05, -0.02, 0.03))
        unit = {"yaw": (0, 0, 1), "pitch": (0, 1, 0), "roll": (1, 0, 0)}[axis]
        half_span = math.radians(axis_spans_deg[axis]) / 2
        for angle in np.linspace(-half_span, half_span, 150):
            body = body_start * Rotation.from_rotvec(np.asarray(unit) * angle)
            pose = sample(pivot_center, body)
            poses.append(PoseSample(pose.position + rng.normal(0, 0.001, 3),
                                    pose.quaternion_xyzw))
        return poses

    sweeps = {axis: sweep(axis, number)
              for number, axis in enumerate(("yaw", "pitch", "roll"), start=1)}
    return (start, forward, left), sweeps, lever_body


def test_recover_upside_down_camera_rotation_and_3d_offset(tmp_path):
    holds, sweeps, lever_body = synthetic_motion()
    mount, fit = mount_from_motion(*holds, axis_sweeps=sweeps)
    assert np.array(mount.body_to_camera_translation_m) == pytest.approx(
        lever_body, abs=0.002)
    expected = Rotation.from_euler("xyz", (math.pi, 0.08, -0.15))
    actual = Rotation.from_euler("xyz", mount.body_to_camera_rpy_rad)
    assert (expected.inv() * actual).magnitude() < 1e-8
    assert fit.rms_m < 0.005
    assert set(fit.axes) == {"yaw", "pitch", "roll"}
    assert all(metrics["span_deg"] > 50 for metrics in fit.axes.values())
    output = tmp_path / "measured_mount.yaml"
    output.write_text(_render_mount(mount), encoding="utf-8")
    loaded = load_mount(output)
    assert loaded.body_to_camera_translation_m == pytest.approx(lever_body, abs=0.002)


def test_yaw_only_pivot_cannot_measure_vertical_offset():
    holds, sweeps, _ = synthetic_motion()
    camera_in_body = camera_orientation_in_body(*holds)
    with pytest.raises(PivotCalibrationError, match="separate yaw, pitch, and roll"):
        fit_axis_pivots({"yaw": sweeps["yaw"]}, camera_in_body)
    assert validate_axis_motion("yaw", sweeps["yaw"], camera_in_body) > 50
    with pytest.raises(PivotCalibrationError, match="pitch: only"):
        validate_axis_motion("pitch", sweeps["yaw"], camera_in_body)
    tiny_poses = []
    rng = np.random.default_rng(23)
    start = holds[0]
    for _ in range(100):
        rotation = Rotation.from_euler("xyz", rng.uniform(-0.02, 0.02, 3))
        tiny_poses.append(PoseSample(
            start.position + rotation.apply((0.2, 0, 0)),
            (rotation * Rotation.from_quat(start.quaternion_xyzw)).as_quat()))
    with pytest.raises(PivotCalibrationError, match="yaw: only"):
        validate_axis_motion("yaw", tiny_poses, camera_in_body)


def test_separate_pivot_centers_allowed_but_moving_one_is_rejected():
    holds, sweeps, _ = synthetic_motion()
    shifted = [PoseSample(pose.position + np.array((0.1, 0, 0)), pose.quaternion_xyzw)
               for pose in sweeps["pitch"]]
    mount, _ = mount_from_motion(*holds, axis_sweeps={**sweeps, "pitch": shifted})
    assert mount.body_to_camera_translation_m[0] == pytest.approx(0.21, abs=0.002)
    drifting = [PoseSample(
        pose.position + np.array((0.2 * i / len(sweeps["roll"]), 0, 0)),
        pose.quaternion_xyzw) for i, pose in enumerate(sweeps["roll"])]
    with pytest.raises(PivotCalibrationError, match="roll: pivot center moved") as exc:
        mount_from_motion(*holds, axis_sweeps={**sweeps, "roll": drifting})
    assert exc.value.axis == "roll"


def test_brief_pivot_tracking_outliers_do_not_discard_good_motion():
    holds, sweeps, lever_body = synthetic_motion()
    pitch_with_outliers = [
        PoseSample(
            pose.position + (np.array((0.12, -0.08, 0.03)) if 40 <= i < 55 else 0),
            pose.quaternion_xyzw)
        for i, pose in enumerate(sweeps["pitch"])
    ]
    mount, fit = mount_from_motion(
        *holds, axis_sweeps={**sweeps, "pitch": pitch_with_outliers})
    assert np.array(mount.body_to_camera_translation_m) == pytest.approx(
        lever_body, abs=0.01)
    assert 0.85 <= fit.axes["pitch"]["inlier_fraction"] < 0.95


def test_modest_separate_rotations_still_identify_the_mount():
    holds, sweeps, lever_body = synthetic_motion(
        {"yaw": 13, "pitch": 11, "roll": 11})
    mount, fit = mount_from_motion(*holds, axis_sweeps=sweeps)
    assert fit.min_rotation_spread >= 0.04
    assert np.array(mount.body_to_camera_translation_m) == pytest.approx(
        lever_body, abs=0.02)


def test_mixed_axis_rotation_is_used_instead_of_rejected():
    holds, sweeps, lever_body = synthetic_motion()
    camera_in_body = Rotation.from_euler("xyz", (math.pi, 0.08, -0.15))
    body_start = Rotation.from_euler("xyz", (0.02, -0.03, 0.4))
    pivot_center = np.array((2.05, -0.52, 0.83))
    mixed_yaw = []
    for angle in np.linspace(-0.5, 0.5, 150):
        body = body_start * Rotation.from_euler("zy", (angle, 0.9 * angle))
        mixed_yaw.append(PoseSample(
            pivot_center + body.apply(lever_body),
            (body * camera_in_body).as_quat()))
    mount, _ = mount_from_motion(*holds, axis_sweeps={**sweeps, "yaw": mixed_yaw})
    assert np.array(mount.body_to_camera_translation_m) == pytest.approx(
        lever_body, abs=0.002)


def test_stable_hold_and_heading_gate():
    holds, _, _ = synthetic_motion()
    start = holds[0]
    assert stable_pose([start] * 12).position == pytest.approx(start.position)
    moving = [PoseSample(start.position + np.array((i * 0.005, 0, 0)),
                         start.quaternion_xyzw) for i in range(20)]
    with pytest.raises(ValueError, match="stationary hold moved"):
        stable_pose(moving)
    rotated_forward = PoseSample(
        holds[1].position,
        (Rotation.from_quat(holds[1].quaternion_xyzw)
         * Rotation.from_euler("z", math.radians(12))).as_quat())
    with pytest.raises(ValueError, match="orientation changed 12.0 deg"):
        mount_from_motion(start, rotated_forward, holds[2],
                          measured_translation_m=(0.21, -0.06, 0.12))


def test_short_imperfect_l_shaped_slides_are_accepted():
    holds, _, _ = synthetic_motion()
    start = holds[0]
    body_in_world = Rotation.from_euler("xyz", (0.02, -0.03, 0.4))
    forward = PoseSample(start.position + body_in_world.apply((0.18, 0, 0)),
                         start.quaternion_xyzw)
    left = PoseSample(forward.position + body_in_world.apply((0.04, 0.18, 0)),
                      start.quaternion_xyzw)
    estimated = camera_orientation_in_body(start, forward, left)
    expected = Rotation.from_euler("xyz", (math.pi, 0.08, -0.15))
    assert (expected.inv() * estimated).magnitude() < math.radians(8)
