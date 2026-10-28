"""Launch-relative flight limits for ZED local-odometry missions."""

import math


LOCAL_POSITION_STATES = frozenset({
    "LOCAL_WARMUP", "LOCAL_SET_OFFBOARD", "LOCAL_ARMING", "LOCAL_TAKEOFF"})


def setpoint_source(state, *, armed):
    """Select exactly one OFFBOARD setpoint family for the local mission."""
    if state in LOCAL_POSITION_STATES:
        return "position"
    if state in {"IDLE", "COMPLETE"} or (state == "ABORT" and not armed):
        return "none"
    return "velocity"


def horizontal_excursion_m(origin_xy, position_xy):
    return math.hypot(position_xy[0] - origin_xy[0],
                      position_xy[1] - origin_xy[1])


def projected_excursion_m(origin_xy, position_xy, yaw_rad, vx_body, vy_body, horizon_s):
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    future_xy = (
        position_xy[0] + horizon_s * (c * vx_body - s * vy_body),
        position_xy[1] + horizon_s * (s * vx_body + c * vy_body),
    )
    return horizontal_excursion_m(origin_xy, future_xy)


def outward_limit_reached(origin_xy, position_xy, yaw_rad, vx_body, vy_body,
                          radius_m, horizon_s):
    current = horizontal_excursion_m(origin_xy, position_xy)
    projected = projected_excursion_m(origin_xy, position_xy, yaw_rad,
                                      vx_body, vy_body, horizon_s)
    return projected > radius_m and projected > current + 1e-6


def pose_discontinuity(previous_xyz, xyz, previous_yaw, yaw, *,
                       max_position_step_m=0.5, max_yaw_step_rad=0.52):
    distance = math.dist(previous_xyz, xyz)
    yaw_step = abs(math.atan2(math.sin(yaw - previous_yaw),
                              math.cos(yaw - previous_yaw)))
    return distance > max_position_step_m or yaw_step > max_yaw_step_rad
