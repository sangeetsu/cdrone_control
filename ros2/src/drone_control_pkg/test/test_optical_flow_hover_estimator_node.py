from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import math
import sys

from builtin_interfaces.msg import Time
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.imu_optical_flow_fusion import FusionPose
from drone_control_pkg.optical_flow_hover_estimator_node import (
    EstimatorHealth,
    effective_flow_range_m,
    evaluate_health,
    flow_gyro_value,
    flow_range_m,
    make_flow_sample,
    make_imu_sample,
    pose_to_msg,
    range_is_valid,
    source_stamp_s,
)


def _stamp(sec: int = 1, nanosec: int = 0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def _header(frame_id: str = "map", sec: int = 1, nanosec: int = 0):
    return SimpleNamespace(frame_id=frame_id, stamp=_stamp(sec, nanosec))


def _vector(x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _quat(x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
    return SimpleNamespace(x=x, y=y, z=z, w=w)


def _imu_msg(**overrides):
    values = {
        "header": _header("base_link", 2, 500000000),
        "orientation": _quat(),
        "angular_velocity": _vector(),
        "linear_acceleration": _vector(z=9.80665),
        "orientation_covariance": [0.1] * 9,
        "angular_velocity_covariance": [0.2] * 9,
        "linear_acceleration_covariance": [0.3] * 9,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def _flow_msg(**overrides):
    values = {
        "header": _header("px4flow", 3, 0),
        "integrated_x": 0.01,
        "integrated_y": 0.02,
        "integrated_xgyro": 0.001,
        "integrated_ygyro": 0.002,
        "integrated_zgyro": 0.003,
        "quality": 100,
        "distance": 1.2,
        "integration_time_us": 20000,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_source_stamp_uses_header_when_present() -> None:
    msg = SimpleNamespace(header=_header(sec=4, nanosec=250000000))

    assert source_stamp_s(msg, 10.0) == pytest.approx(4.25)


def test_source_stamp_falls_back_for_zero_stamp() -> None:
    msg = SimpleNamespace(header=_header(sec=0, nanosec=0))

    assert source_stamp_s(msg, 10.0) == pytest.approx(10.0)


def test_range_is_valid_uses_configured_window() -> None:
    assert range_is_valid(0.5, range_min_m=0.3, range_max_m=5.0)
    assert not range_is_valid(0.2, range_min_m=0.3, range_max_m=5.0)
    assert not range_is_valid(float("nan"), range_min_m=0.3, range_max_m=5.0)


def test_flow_range_prefers_optical_flow_distance() -> None:
    latest_range = SimpleNamespace(range=0.7)

    assert flow_range_m(_flow_msg(distance=1.2), latest_range) == pytest.approx(1.2)


def test_flow_range_falls_back_to_range_topic() -> None:
    latest_range = SimpleNamespace(range=0.7)

    assert flow_range_m(_flow_msg(distance=0.0), latest_range) == pytest.approx(0.7)


def test_effective_flow_range_clamps_near_ground_range() -> None:
    assert effective_flow_range_m(
        0.0,
        range_min_m=0.05,
        near_ground_range_m=0.05,
        allow_near_ground_flow=True,
    ) == pytest.approx(0.05)
    assert effective_flow_range_m(
        0.03,
        range_min_m=0.05,
        near_ground_range_m=0.05,
        allow_near_ground_flow=True,
    ) == pytest.approx(0.05)


def test_effective_flow_range_preserves_rejection_when_disabled() -> None:
    assert effective_flow_range_m(
        0.03,
        range_min_m=0.05,
        near_ground_range_m=0.05,
        allow_near_ground_flow=False,
    ) == pytest.approx(0.03)


def test_missing_flow_gyro_can_fall_back_to_zero() -> None:
    assert flow_gyro_value(
        float("nan"),
        allow_missing_flow_gyro=True,
    ) == pytest.approx(0.0)


def test_missing_flow_gyro_preserves_nan_when_disabled() -> None:
    assert math.isnan(
        flow_gyro_value(
            float("nan"),
            allow_missing_flow_gyro=False,
        )
    )


def test_make_imu_sample_copies_motion_and_covariance() -> None:
    msg = _imu_msg(
        angular_velocity=_vector(x=0.1, y=0.2, z=0.3),
        linear_acceleration=_vector(x=1.0, y=2.0, z=9.8),
    )

    sample = make_imu_sample(msg, source_s=2.5, receive_s=2.6)

    assert sample.stamp_s == pytest.approx(2.5)
    assert sample.receive_stamp_s == pytest.approx(2.6)
    assert sample.frame_id == "base_link"
    assert sample.angular_velocity_z == pytest.approx(0.3)
    assert sample.linear_acceleration_x == pytest.approx(1.0)
    assert sample.linear_acceleration_covariance == tuple([0.3] * 9)


def test_make_flow_sample_copies_integrated_flow_and_range() -> None:
    sample = make_flow_sample(
        _flow_msg(),
        range_m=1.4,
        source_s=3.0,
        receive_s=3.1,
        sequence=7,
        range_variance_m2=0.01,
    )

    assert sample.integrated_x == pytest.approx(0.01)
    assert sample.integrated_y == pytest.approx(0.02)
    assert sample.distance_m == pytest.approx(1.4)
    assert sample.integration_time_s == pytest.approx(0.02)
    assert sample.sequence == 7
    assert sample.range_variance_m2 == pytest.approx(0.01)


def test_make_flow_sample_can_sanitize_missing_gyro() -> None:
    sample = make_flow_sample(
        _flow_msg(
            integrated_xgyro=float("nan"),
            integrated_ygyro=float("nan"),
            integrated_zgyro=float("nan"),
        ),
        range_m=1.4,
        source_s=3.0,
        receive_s=3.1,
        sequence=7,
        range_variance_m2=0.01,
        allow_missing_flow_gyro=True,
    )

    assert sample.integrated_xgyro == pytest.approx(0.0)
    assert sample.integrated_ygyro == pytest.approx(0.0)
    assert sample.integrated_zgyro == pytest.approx(0.0)


def test_health_rejects_before_valid_flow_or_warmup() -> None:
    health = evaluate_health(
        now_s=10.0,
        initialized=True,
        last_imu_receive_s=9.95,
        last_range_receive_s=9.95,
        last_flow_receive_s=9.95,
        last_valid_flow_receive_s=None,
        last_flow_quality=100,
        latest_range_m=1.0,
        latest_flow_update=None,
        init_time_s=9.8,
        max_imu_age_s=0.25,
        max_range_age_s=0.25,
        max_flow_age_s=0.5,
        quality_min=10,
        range_min_m=0.3,
        range_max_m=5.0,
        allow_imu_only_warmup=True,
        imu_only_warmup_s=1.0,
    )

    assert health == EstimatorHealth(False, "flow_not_accepted")


def test_health_accepts_recent_valid_flow() -> None:
    health = evaluate_health(
        now_s=10.0,
        initialized=True,
        last_imu_receive_s=9.95,
        last_range_receive_s=9.95,
        last_flow_receive_s=9.95,
        last_valid_flow_receive_s=9.95,
        last_flow_quality=100,
        latest_range_m=1.0,
        latest_flow_update=None,
        init_time_s=9.0,
        max_imu_age_s=0.25,
        max_range_age_s=0.25,
        max_flow_age_s=0.5,
        quality_min=10,
        range_min_m=0.3,
        range_max_m=5.0,
        allow_imu_only_warmup=True,
        imu_only_warmup_s=1.0,
    )

    assert health == EstimatorHealth(True, "ready")


def test_health_rejects_low_quality_flow() -> None:
    health = evaluate_health(
        now_s=10.0,
        initialized=True,
        last_imu_receive_s=9.95,
        last_range_receive_s=9.95,
        last_flow_receive_s=9.95,
        last_valid_flow_receive_s=9.95,
        last_flow_quality=5,
        latest_range_m=1.0,
        latest_flow_update=None,
        init_time_s=9.0,
        max_imu_age_s=0.25,
        max_range_age_s=0.25,
        max_flow_age_s=0.5,
        quality_min=10,
        range_min_m=0.3,
        range_max_m=5.0,
        allow_imu_only_warmup=True,
        imu_only_warmup_s=1.0,
    )

    assert health == EstimatorHealth(False, "flow_quality_below_min")


def test_pose_to_msg_publishes_yaw_quaternion() -> None:
    pose = FusionPose(
        x_m=1.0,
        y_m=2.0,
        z_m=0.8,
        vx_mps=0.0,
        vy_mps=0.0,
        vz_mps=0.0,
        yaw_rad=math.pi / 2.0,
        frame_id="map",
    )

    msg = pose_to_msg(pose, Time(sec=5, nanosec=0))

    assert msg.header.frame_id == "map"
    assert msg.pose.position.x == pytest.approx(1.0)
    assert msg.pose.orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert msg.pose.orientation.w == pytest.approx(math.cos(math.pi / 4.0))


def test_launch_file_is_mocap_free_and_bridges_estimator_output() -> None:
    launch_path = (
        Path(__file__).resolve().parents[2]
        / "drone_bringup"
        / "launch"
        / "of_position_hover_demo.launch.py"
    )
    content = launch_path.read_text(encoding="utf-8")

    assert "vrpn_mocap" not in content
    assert "external_pose_adapter_node" not in content
    assert "optical_flow_hover_estimator_node" in content
    assert '"input_pose_topic": external_pose_topic' in content
    assert '"local_pose_topic": LaunchConfiguration("demo_local_pose_topic")' in content
    assert (
        'DeclareLaunchArgument("require_companion_active", default_value="false")'
        in content
    )
