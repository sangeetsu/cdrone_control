from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.minjerk_waypoint_utils import (
    TrajectoryPose,
    build_minjerk_segments,
    load_waypoint_mission,
    minimum_jerk_blend,
    resolve_waypoints_for_start,
    sample_minjerk_segment,
)
from drone_control_pkg.perimeter_utils import PerimeterGuard


def test_minimum_jerk_blend_has_smooth_endpoints() -> None:
    assert minimum_jerk_blend(0.0) == pytest.approx(0.0)
    assert minimum_jerk_blend(1.0) == pytest.approx(1.0)
    assert minimum_jerk_blend(0.5) == pytest.approx(0.5)


def test_waypoint_yaml_and_segment_sampling(tmp_path: Path) -> None:
    config_path = tmp_path / "waypoints.yaml"
    config_path.write_text(
        """
name: test
frame_id: map
defaults:
  cruise_speed_mps: 0.5
  min_segment_duration_s: 2.0
  hold_s: 0.25
  final_hold_s: 1.0
waypoints:
  - name: first
    x_m: 1.0
    y_m: 0.0
    z_m: 1.2
    yaw_deg: 0.0
  - name: second
    x_m: 1.0
    y_m: 1.0
    z_m: 1.2
    yaw_deg: 90.0
    duration_s: 3.0
""",
        encoding="utf-8",
    )

    mission = load_waypoint_mission(str(config_path))
    segments = build_minjerk_segments(
        TrajectoryPose(0.0, 0.0, 1.2, 0.0),
        mission,
    )

    assert mission.route.start_policy == "fixed"
    assert not mission.route.loop
    assert not mission.route.return_to_start
    assert len(segments) == 2
    assert segments[0].duration_s == pytest.approx(2.0)
    assert segments[1].duration_s == pytest.approx(3.0)
    assert segments[-1].hold_s == pytest.approx(1.0)

    midpoint = sample_minjerk_segment(segments[0], 1.0)
    assert midpoint.x_m == pytest.approx(0.5)
    assert midpoint.y_m == pytest.approx(0.0)


def test_yaw_uses_shortest_unwrapped_path(tmp_path: Path) -> None:
    config_path = tmp_path / "yaw.yaml"
    config_path.write_text(
        """
frame_id: map
waypoints:
  - x_m: 0.0
    y_m: 0.0
    z_m: 1.0
    yaw_deg: -170.0
""",
        encoding="utf-8",
    )

    mission = load_waypoint_mission(str(config_path))
    segments = build_minjerk_segments(
        TrajectoryPose(0.0, 0.0, 1.0, math.radians(170.0)),
        mission,
    )

    assert math.degrees(segments[0].end.yaw_rad) == pytest.approx(190.0)


def test_route_metadata_resolves_nearest_reachable_loop(
    tmp_path: Path,
) -> None:
    config_path = tmp_path / "route.yaml"
    config_path.write_text(
        """
frame_id: map
route:
  start_policy: nearest_reachable
  loop: true
  return_to_start: true
waypoints:
  - name: west
    x_m: 0.0
    y_m: 0.0
    z_m: 1.8
  - name: center
    x_m: 5.0
    y_m: 0.0
    z_m: 1.8
  - name: east
    x_m: 10.0
    y_m: 0.0
    z_m: 1.8
""",
        encoding="utf-8",
    )

    mission = load_waypoint_mission(str(config_path))
    start_pose = TrajectoryPose(9.0, 1.0, 1.2, 0.0)
    return_pose = TrajectoryPose(2.5, -3.0, 1.2, 0.0)
    resolved = resolve_waypoints_for_start(
        start_pose,
        mission,
        waypoint_reachable=lambda _start, _waypoint: None,
        return_pose=return_pose,
    )

    assert mission.route.start_policy == "nearest_reachable"
    assert mission.route.loop
    assert mission.route.return_to_start
    assert [waypoint.name for waypoint in resolved] == [
        "east",
        "west",
        "center",
        "east",
        "return_to_start",
    ]
    assert resolved[-1].x_m == pytest.approx(return_pose.x_m)
    assert resolved[-1].y_m == pytest.approx(return_pose.y_m)
    assert resolved[-1].z_m == pytest.approx(1.8)


def test_nearest_reachable_route_rejects_unreachable_start(
    tmp_path: Path,
) -> None:
    config_path = tmp_path / "unreachable.yaml"
    config_path.write_text(
        """
frame_id: map
route:
  start_policy: nearest_reachable
waypoints:
  - name: blocked
    x_m: 1.0
    y_m: 0.0
    z_m: 1.8
""",
        encoding="utf-8",
    )

    mission = load_waypoint_mission(str(config_path))
    with pytest.raises(ValueError, match="no reachable waypoint entry"):
        resolve_waypoints_for_start(
            TrajectoryPose(0.0, 0.0, 1.2, 0.0),
            mission,
            waypoint_reachable=lambda _start, _waypoint: "blocked",
        )


def test_default_studio_sweep_is_perimeter_safe() -> None:
    source_root = Path(__file__).resolve().parents[2]
    mission = load_waypoint_mission(
        str(source_root / "drone_bringup/config/minjerk_waypoints.yaml")
    )
    guard = PerimeterGuard.load_from_yaml(
        str(source_root / "drone_bringup/config/drone_studio_perimeter.yaml")
    )

    def reachable(start: TrajectoryPose, waypoint) -> str | None:
        return guard.segment_violation_reason(
            (start.x_m, start.y_m),
            (waypoint.x_m, waypoint.y_m),
            sample_step_m=0.10,
            boundary_tolerance_m=0.05,
            boundary_margin_m=1.0,
            keep_out_margin_m=0.0,
        )

    starts = (
        TrajectoryPose(-11.0, 0.0, 1.8, 0.0),
        TrajectoryPose(2.0, 6.5, 1.8, 0.0),
        TrajectoryPose(-9.2, 7.2, 1.8, 0.0),
        TrajectoryPose(-2.0, 7.8, 1.8, 0.0),
    )

    assert mission.route.start_policy == "nearest_reachable"
    assert mission.route.loop
    assert mission.route.return_to_start
    for start in starts:
        assert (
            guard.xy_violation_reason(
                start.x_m,
                start.y_m,
                boundary_tolerance_m=0.05,
                boundary_margin_m=1.0,
                keep_out_margin_m=0.0,
            )
            is None
        )
        resolved = resolve_waypoints_for_start(
            start,
            mission,
            waypoint_reachable=reachable,
        )
        assert len(resolved) == len(mission.waypoints) + 2
        prior_xy = (start.x_m, start.y_m)
        for waypoint in resolved:
            waypoint_reason = guard.xy_violation_reason(
                waypoint.x_m,
                waypoint.y_m,
                boundary_tolerance_m=0.05,
                boundary_margin_m=1.0,
                keep_out_margin_m=0.0,
            )
            assert waypoint_reason is None, waypoint.name
            assert guard.goal_altitude_violation_reason(waypoint.z_m) is None
            path_reason = guard.segment_violation_reason(
                prior_xy,
                (waypoint.x_m, waypoint.y_m),
                sample_step_m=0.10,
                boundary_tolerance_m=0.05,
                boundary_margin_m=1.0,
                keep_out_margin_m=0.0,
            )
            assert path_reason is None, waypoint.name
            prior_xy = (waypoint.x_m, waypoint.y_m)
