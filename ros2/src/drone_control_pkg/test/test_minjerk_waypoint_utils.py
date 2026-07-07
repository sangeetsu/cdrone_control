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
    sample_minjerk_segment,
)


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
