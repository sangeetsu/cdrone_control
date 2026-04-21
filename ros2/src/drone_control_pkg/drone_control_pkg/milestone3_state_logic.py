from __future__ import annotations


POST_LANDING_CLEANUP_STATES = {
    "RESTORE_TAKEOFF_PARAM",
    "RESTORE_SPEED_PROFILE",
}


def requires_runtime_tracking_guards(demo_state: str, *, armed: bool) -> bool:
    return not (
        str(demo_state) in POST_LANDING_CLEANUP_STATES and not bool(armed)
    )
