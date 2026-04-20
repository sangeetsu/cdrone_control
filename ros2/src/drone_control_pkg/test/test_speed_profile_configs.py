from pathlib import Path

from drone_control_pkg.px4_param_profile import Px4ParamProfile


def _bringup_config_dir() -> Path:
    src_dir = Path(__file__).resolve().parents[2]
    return src_dir / "drone_bringup" / "config"


def test_indoor_speed_profile_does_not_override_takeoff_speed():
    profile = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "indoor_speed_profile.yaml")
    )

    assert "MPC_TKO_SPEED" not in profile.parameters
    assert profile.parameters["MPC_XY_CRUISE"] == 1.0
    assert profile.parameters["MPC_LAND_SPEED"] == 0.35


def test_milestone3_takeoff_baseline_profile_restores_takeoff_speed():
    profile = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "milestone3_takeoff_baseline_profile.yaml")
    )

    assert profile.parameters["MPC_TKO_SPEED"] == 1.5
    assert profile.parameters["MPC_XY_CRUISE"] == 5.0
    assert profile.parameters["MPC_LAND_SPEED"] == 0.7
