from pathlib import Path
import sys


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg import zed_local_combined_node


def test_startup_stage_is_saved_outside_ros_launch_log(tmp_path, monkeypatch, capsys):
    trace = tmp_path / "startup.log"
    monkeypatch.setenv("MILESTONE4_STARTUP_TRACE", str(trace))
    monkeypatch.setattr(zed_local_combined_node, "_power_snapshot",
                        lambda: "oc1_events=3 vdd_in_mv=4800")

    zed_local_combined_node._startup_stage("opening ZED")

    saved = trace.read_text()
    assert "opening ZED" in saved
    assert "oc1_events=3 vdd_in_mv=4800" in saved
    assert saved in capsys.readouterr().err


def test_power_preflight_rejects_low_module_input(monkeypatch):
    monkeypatch.setattr(zed_local_combined_node, "read_jetson_power",
                        lambda: (174838, 4632))
    issue = zed_local_combined_node.check_input_power(sample_s=0)
    assert "4632 mV" in issue
    assert "below 4850 mV" in issue


def test_power_preflight_rejects_rising_undervoltage_counter(monkeypatch):
    readings = iter([(10, 4932), (14, 4932)])
    monkeypatch.setattr(zed_local_combined_node, "read_jetson_power",
                        lambda: next(readings))
    monkeypatch.setattr(zed_local_combined_node.time, "sleep", lambda _: None)
    assert "rose from 10 to 14" in zed_local_combined_node.check_input_power()


def test_power_preflight_allows_stable_supply(monkeypatch):
    readings = iter([(10, 4932), (10, 4940)])
    monkeypatch.setattr(zed_local_combined_node, "read_jetson_power",
                        lambda: next(readings))
    monkeypatch.setattr(zed_local_combined_node.time, "sleep", lambda _: None)
    assert zed_local_combined_node.check_input_power() == ""


def test_power_preflight_rejects_missing_telemetry(monkeypatch):
    monkeypatch.setattr(zed_local_combined_node, "read_jetson_power",
                        lambda: (None, None))
    assert "telemetry unavailable" in zed_local_combined_node.check_input_power()
