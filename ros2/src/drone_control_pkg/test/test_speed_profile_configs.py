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


def test_default_speed_profile_matches_faster_baseline_motion_limits():
    profile = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "default_speed_profile.yaml")
    )

    assert profile.parameters["MPC_XY_CRUISE"] == 5.0
    assert profile.parameters["MPC_XY_VEL_MAX"] == 12.0
    assert profile.parameters["MPC_LAND_SPEED"] == 0.7


def test_fun_speed_profile_sits_between_indoor_and_default():
    indoor = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "indoor_speed_profile.yaml")
    )
    fun = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "fun_speed_profile.yaml")
    )
    default = Px4ParamProfile.load_from_yaml(
        str(_bringup_config_dir() / "default_speed_profile.yaml")
    )

    for key in (
        "MPC_XY_CRUISE",
        "MPC_XY_VEL_MAX",
        "MPC_ACC_HOR",
        "MPC_JERK_AUTO",
        "MPC_JERK_MAX",
        "MPC_Z_V_AUTO_UP",
        "MPC_Z_V_AUTO_DN",
        "MPC_Z_VEL_MAX_DN",
        "MPC_LAND_SPEED",
    ):
        assert indoor.parameters[key] < fun.parameters[key] < default.parameters[key]
