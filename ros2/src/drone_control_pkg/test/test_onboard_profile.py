import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest
import yaml

from drone_control_pkg.onboard_hover_logic import HoverSettings


REPO = Path(__file__).resolve().parents[4]
spec = importlib.util.spec_from_file_location(
    "onboard_inspector", REPO / "scripts/inspect_onboard_localization.py")
inspector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(inspector)


def test_px4_bytewise_integers_are_decoded_before_diff_and_backup():
    for actual in (0, 1, 4, -1, 16777217):
        raw = struct.unpack("<f", struct.pack("<i", actual))[0]
        msg = SimpleNamespace(param_type=6, param_value=raw)
        assert inspector.decode_parameter(msg, True) == actual
    assert inspector.decode_parameter(SimpleNamespace(param_type=6, param_value=4.0), False) == 4


def test_profile_preserves_calibration_offsets_and_gains():
    profile = inspector.load_parameters(inspector.DEFAULT_PROFILE)
    assert profile["UAVCAN_SUB_FLOW"] == profile["UAVCAN_SUB_RNG"] == 1
    assert profile["EKF2_EV_CTRL"] == profile["EKF2_GPS_CTRL"] == 0
    assert profile["EKF2_HGT_REF"] == profile["EKF2_RNG_CTRL"] == 2
    assert profile["EKF2_BARO_CTRL"] == 1
    assert profile["COM_OBL_RC_ACT"] == 4
    assert not any(name.startswith(("CAL_", "MPC_")) or "POS_" in name for name in profile)
    assert "SENS_FLOW_ROT" not in profile


def test_offline_commands_roundtrip_rollback_values(tmp_path):
    original = {"UAVCAN_SUB_FLOW": 0, "EKF2_OF_POS_X": 0.06}
    path = tmp_path / "backup.yaml"
    path.write_text(yaml.safe_dump({"parameters": original}))
    assert inspector.load_parameters(path) == original
    commands = inspector.parameter_commands(original)
    assert "param set UAVCAN_SUB_FLOW 0" in commands
    assert "reboot" not in commands


def test_command_output_rejects_untrusted_parameter_names(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump({"parameters": {"BAD; reboot": 1}}))
    with pytest.raises(ValueError):
        inspector.load_parameters(path)


def test_onboard_overlay_excludes_pose_injection_and_filters_downward_range():
    path = REPO / "ros2/src/drone_bringup/config/onboard_mavros.yaml"
    config = yaml.safe_load(path.read_text())
    denied = config["/**"]["ros__parameters"]["plugin_denylist"]
    assert {"fake_gps", "mocap_pose_estimate", "vision_pose", "vision_speed"}.issubset(denied)
    sensor = yaml.safe_load(config["/**/distance_sensor"]["ros__parameters"]["config"])
    assert sensor["ark_flow"]["orientation"] == "PITCH_270"
    flow = config["/**/px4flow"]["ros__parameters"]
    profile = inspector.load_parameters(inspector.DEFAULT_PROFILE)
    assert flow["ranger_min_range"] == HoverSettings().range_min_m == 0.05
    assert profile["UAVCAN_RNG_MIN"] == profile["SENS_FLOW_MINHGT"] == 0.08
    assert flow["ranger_max_range"] == profile["UAVCAN_RNG_MAX"] == 30.0


def test_zed_profile_enables_only_vision_aiding_and_overlay_allows_it():
    profile = inspector.load_parameters(
        REPO / "ros2/src/drone_bringup/config/zed_hover_px4_1_16.yaml")
    assert profile["EKF2_EV_CTRL"] == 11
    assert profile["EKF2_HGT_REF"] == 3
    assert profile["EKF2_OF_CTRL"] == profile["EKF2_RNG_CTRL"] == 0
    assert profile["EKF2_EV_POS_X"] == profile["EKF2_EV_POS_Y"] == profile["EKF2_EV_POS_Z"] == 0
    config = yaml.safe_load((REPO / "ros2/src/drone_bringup/config/zed_mavros.yaml").read_text())
    denied = config["/**"]["ros__parameters"]["plugin_denylist"]
    assert "vision_pose" not in denied
    assert "mocap_pose_estimate" in denied
