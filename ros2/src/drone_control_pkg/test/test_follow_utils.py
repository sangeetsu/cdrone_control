from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.follow_utils import (
    FollowCommand,
    clamp_follow_command_altitude,
    compute_return_to_point_command,
)


def make_command(*, vz: float) -> FollowCommand:
    return FollowCommand(
        vx=0.0,
        vy=0.0,
        vz=vz,
        yaw_rate=0.0,
        err_forward=0.0,
        err_lateral=0.0,
        err_vertical=0.0,
        yaw_error=0.0,
        min_distance_gate_active=False,
    )


def test_clamp_follow_command_altitude_limits_descent_at_floor() -> None:
    command = make_command(vz=-0.4)

    clamped = clamp_follow_command_altitude(
        command,
        current_altitude_m=0.42,
        projected_horizon_s=1.0,
        min_z_m=0.35,
        max_z_m=2.64,
    )

    assert clamped.vz == pytest.approx(-0.07)


def test_clamp_follow_command_altitude_preserves_safe_command() -> None:
    command = make_command(vz=0.1)

    clamped = clamp_follow_command_altitude(
        command,
        current_altitude_m=1.0,
        projected_horizon_s=1.0,
        min_z_m=0.35,
        max_z_m=2.64,
    )

    assert clamped == command


def test_clamp_follow_command_altitude_limits_ascent_at_ceiling() -> None:
    command = make_command(vz=0.6)

    clamped = clamp_follow_command_altitude(
        command,
        current_altitude_m=2.5,
        projected_horizon_s=1.0,
        min_z_m=0.35,
        max_z_m=2.64,
    )

    assert clamped.vz == pytest.approx(0.14)


def test_compute_return_to_point_command_moves_forward_toward_world_x_target() -> None:
    command = compute_return_to_point_command(
        current_xy=(0.0, 0.0),
        current_altitude_m=2.0,
        current_yaw_rad=0.0,
        target_xy=(1.0, 0.0),
        target_altitude_m=2.0,
        target_yaw_rad=0.0,
        xy_deadband_m=0.05,
        z_deadband_m=0.05,
        yaw_deadband_rad=0.05,
        kp_xy=0.5,
        kp_z=0.3,
        kp_yaw=0.8,
        max_vel_xy_mps=0.6,
        max_vel_z_mps=0.3,
        max_yaw_rate_rps=0.4,
    )

    assert command.vx == pytest.approx(0.5)
    assert command.vy == pytest.approx(0.0)
    assert command.vz == pytest.approx(0.0)
    assert command.yaw_rate == pytest.approx(0.0)


def test_compute_return_to_point_command_rotates_world_error_into_body_frame() -> None:
    command = compute_return_to_point_command(
        current_xy=(0.0, 0.0),
        current_altitude_m=2.0,
        current_yaw_rad=1.5707963267948966,
        target_xy=(1.0, 0.0),
        target_altitude_m=2.3,
        target_yaw_rad=0.0,
        xy_deadband_m=0.05,
        z_deadband_m=0.05,
        yaw_deadband_rad=0.05,
        kp_xy=0.5,
        kp_z=0.3,
        kp_yaw=0.8,
        max_vel_xy_mps=0.6,
        max_vel_z_mps=0.3,
        max_yaw_rate_rps=0.4,
    )

    assert command.vx == pytest.approx(0.0, abs=1e-6)
    assert command.vy == pytest.approx(-0.5)
    assert command.vz == pytest.approx(0.09)
    assert command.yaw_rate == pytest.approx(-0.4)
