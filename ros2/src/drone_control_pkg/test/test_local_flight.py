from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.local_flight import (
    horizontal_excursion_m, outward_limit_reached, pose_discontinuity,
    projected_excursion_m, setpoint_source,
)


def test_position_and_velocity_setpoint_phases_do_not_overlap():
    for state in ("LOCAL_WARMUP", "LOCAL_SET_OFFBOARD", "LOCAL_ARMING", "LOCAL_TAKEOFF"):
        assert setpoint_source(state, armed=True) == "position"
    for state in ("LOCAL_HANDOFF", "STAGE_HOVER", "FOLLOW", "RETURN_TO_START"):
        assert setpoint_source(state, armed=True) == "velocity"
    assert setpoint_source("IDLE", armed=False) == "none"
    assert setpoint_source("ABORT", armed=False) == "none"


def test_local_radius_uses_takeoff_origin_and_current_heading():
    origin = (4.0, -3.0)
    current = (23.4, -3.0)
    assert horizontal_excursion_m(origin, current) == 19.4
    assert outward_limit_reached(origin, current, 0.0, 1.2, 0.0, 20.0, 0.75)
    assert not outward_limit_reached(origin, current, 0.0, -1.2, 0.0, 20.0, 0.75)
    assert math.isclose(projected_excursion_m(origin, current, math.pi / 2,
                                               1.0, 0.0, 1.0),
                        math.hypot(19.4, 1.0))


def test_pose_jump_is_distinct_from_continuous_motion():
    assert not pose_discontinuity((0, 0, 0), (0.1, 0.0, 0), 0.0, 0.02)
    assert pose_discontinuity((0, 0, 0), (1.0, 0.0, 0), 0.0, 0.02)
    assert pose_discontinuity((0, 0, 0), (0.1, 0.0, 0), 0.0, 1.0)
