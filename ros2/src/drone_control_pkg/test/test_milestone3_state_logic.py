from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.milestone3_state_logic import (
    completion_next_state,
    post_return_next_state,
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


def test_completion_prefers_return_to_start_when_configured() -> None:
    assert (
        completion_next_state(
            return_to_takeoff_on_complete=True,
            has_takeoff_return_target=True,
            land_on_complete=True,
        )
        == "RETURN_TO_START"
    )


def test_completion_falls_back_to_landing_without_return_target() -> None:
    assert (
        completion_next_state(
            return_to_takeoff_on_complete=True,
            has_takeoff_return_target=False,
            land_on_complete=True,
        )
        == "SET_LAND_MODE"
    )


def test_post_return_next_state_honors_land_setting() -> None:
    assert post_return_next_state(land_on_complete=True) == "SET_LAND_MODE"
    assert post_return_next_state(land_on_complete=False) == "COMPLETE"
