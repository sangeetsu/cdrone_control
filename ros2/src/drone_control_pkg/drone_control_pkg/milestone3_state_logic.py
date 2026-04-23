from __future__ import annotations


POST_LANDING_CLEANUP_STATES = {
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}


def completion_next_state(
    *,
    return_to_takeoff_on_complete: bool,
    has_takeoff_return_target: bool,
    land_on_complete: bool,
) -> str:
    if return_to_takeoff_on_complete and has_takeoff_return_target:
        return "RETURN_TO_START"
    if land_on_complete:
        return "SET_LAND_MODE"
    return "COMPLETE"


def post_return_next_state(*, land_on_complete: bool) -> str:
    return "SET_LAND_MODE" if land_on_complete else "COMPLETE"


def requires_runtime_tracking_guards(demo_state: str, *, armed: bool) -> bool:
    return not (
        str(demo_state) in POST_LANDING_CLEANUP_STATES and not bool(armed)
    )
