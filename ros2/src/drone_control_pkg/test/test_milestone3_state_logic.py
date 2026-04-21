from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.milestone3_state_logic import (
    requires_runtime_tracking_guards,
)


def test_runtime_tracking_guards_stay_enabled_in_flight_states() -> None:
    assert requires_runtime_tracking_guards("FOLLOW", armed=True) is True
    assert requires_runtime_tracking_guards("WAIT_TOUCHDOWN", armed=True) is True
    assert requires_runtime_tracking_guards("DISARMING", armed=True) is True


def test_runtime_tracking_guards_disable_only_for_post_landing_cleanup() -> None:
    assert requires_runtime_tracking_guards("RESTORE_SPEED_PROFILE", armed=False) is False
    assert requires_runtime_tracking_guards("RESTORE_TAKEOFF_PARAM", armed=False) is False
    assert requires_runtime_tracking_guards("RESTORE_SPEED_PROFILE", armed=True) is True
