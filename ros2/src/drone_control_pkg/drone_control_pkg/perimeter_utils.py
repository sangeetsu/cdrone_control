from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import yaml


def _distance_point_to_segment(
    point_xy: tuple[float, float],
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> float:
    px, py = point_xy
    ax, ay = start_xy
    bx, by = end_xy
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay

    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - ax, py - ay)

    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - bx, py - by)

    t = c1 / c2
    proj_x = ax + (t * vx)
    proj_y = ay + (t * vy)
    return math.hypot(px - proj_x, py - proj_y)


def _point_in_polygon(
    point_xy: tuple[float, float],
    vertices_xy: tuple[tuple[float, float], ...],
    *,
    edge_tolerance_m: float = 0.0,
) -> bool:
    x, y = point_xy
    tolerance = max(edge_tolerance_m, 0.0)

    for idx in range(len(vertices_xy)):
        start_xy = vertices_xy[idx]
        end_xy = vertices_xy[(idx + 1) % len(vertices_xy)]
        if _distance_point_to_segment(point_xy, start_xy, end_xy) <= tolerance:
            return True

    inside = False
    j = len(vertices_xy) - 1
    for i in range(len(vertices_xy)):
        xi, yi = vertices_xy[i]
        xj, yj = vertices_xy[j]
        crosses = ((yi > y) != (yj > y)) and (
            x < ((xj - xi) * (y - yi) / ((yj - yi) + 1e-12)) + xi
        )
        if crosses:
            inside = not inside
        j = i
    return inside


def _distance_to_polygon_edges(
    point_xy: tuple[float, float],
    vertices_xy: tuple[tuple[float, float], ...],
) -> float:
    return min(
        _distance_point_to_segment(
            point_xy,
            vertices_xy[idx],
            vertices_xy[(idx + 1) % len(vertices_xy)],
        )
        for idx in range(len(vertices_xy))
    )


@dataclass(frozen=True)
class PolygonRegion:
    name: str
    vertices_xy_m: tuple[tuple[float, float], ...]


@dataclass(frozen=True)
class AltitudeLimits:
    min_z_m: Optional[float]
    max_z_m: Optional[float]


@dataclass(frozen=True)
class PerimeterGuard:
    frame_id: str
    boundary: PolygonRegion
    keep_outs: tuple[PolygonRegion, ...]
    altitude_limits: AltitudeLimits
    source_path: str

    @classmethod
    def load_from_yaml(cls, path: str) -> "PerimeterGuard":
        config_path = Path(path).expanduser()
        if not config_path.is_file():
            raise FileNotFoundError(f"perimeter config not found: {config_path}")

        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError(f"expected mapping in perimeter config: {config_path}")

        frame_id = str(loaded.get("frame_id", "")).strip() or "map"
        boundary = PolygonRegion(
            name="boundary",
            vertices_xy_m=_parse_vertices(
                loaded.get("boundary_polygon_xy_m"),
                region_name="boundary_polygon_xy_m",
            ),
        )
        keep_outs_raw = loaded.get("keep_out_polygons_xy_m") or []
        if not isinstance(keep_outs_raw, list):
            raise ValueError("keep_out_polygons_xy_m must be a list")
        keep_outs: list[PolygonRegion] = []
        for item in keep_outs_raw:
            if not isinstance(item, dict):
                raise ValueError("each keep-out polygon must be a mapping")
            keep_out_name = str(item.get("name", "")).strip() or "keep_out"
            keep_outs.append(
                PolygonRegion(
                    name=keep_out_name,
                    vertices_xy_m=_parse_vertices(
                        item.get("vertices_xy_m"),
                        region_name=f"keep_out_polygons_xy_m[{keep_out_name}]",
                    ),
                )
            )

        altitude_limits_raw = loaded.get("altitude_limits_m") or {}
        if not isinstance(altitude_limits_raw, dict):
            raise ValueError("altitude_limits_m must be a mapping")
        altitude_limits = AltitudeLimits(
            min_z_m=_optional_float(altitude_limits_raw.get("min_z_m")),
            max_z_m=_optional_float(altitude_limits_raw.get("max_z_m")),
        )

        return cls(
            frame_id=frame_id,
            boundary=boundary,
            keep_outs=tuple(keep_outs),
            altitude_limits=altitude_limits,
            source_path=str(config_path),
        )

    def frame_matches(self, frame_id: str) -> bool:
        expected = str(self.frame_id or "").strip()
        actual = str(frame_id or "").strip()
        if not expected or not actual:
            return True
        return expected == actual

    def xy_violation_reason(
        self,
        x_m: float,
        y_m: float,
        *,
        boundary_tolerance_m: float = 0.0,
        boundary_margin_m: float = 0.0,
        keep_out_margin_m: float = 0.0,
    ) -> Optional[str]:
        point_xy = (float(x_m), float(y_m))
        if not _point_in_polygon(
            point_xy,
            self.boundary.vertices_xy_m,
            edge_tolerance_m=max(boundary_tolerance_m, 0.0),
        ):
            return "outside studio boundary"
        minimum_clearance_m = max(boundary_margin_m, 0.0)
        if minimum_clearance_m > 0.0:
            boundary_clearance_m = _distance_to_polygon_edges(
                point_xy,
                self.boundary.vertices_xy_m,
            )
            if boundary_clearance_m < minimum_clearance_m:
                return (
                    "inside minimum boundary clearance "
                    f"({boundary_clearance_m:.2f} m < {minimum_clearance_m:.2f} m)"
                )
        for keep_out in self.keep_outs:
            if _point_in_polygon(
                point_xy,
                keep_out.vertices_xy_m,
                edge_tolerance_m=max(keep_out_margin_m, 0.0),
            ):
                return f"inside keep-out polygon '{keep_out.name}'"
        return None

    def segment_violation_reason(
        self,
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
        *,
        sample_step_m: float = 0.10,
        boundary_tolerance_m: float = 0.0,
        boundary_margin_m: float = 0.0,
        keep_out_margin_m: float = 0.0,
    ) -> Optional[str]:
        step_m = max(sample_step_m, 1e-3)
        dx = float(end_xy[0] - start_xy[0])
        dy = float(end_xy[1] - start_xy[1])
        distance_m = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(distance_m / step_m)))
        for step_idx in range(steps + 1):
            t = step_idx / steps
            sample_x = float(start_xy[0] + (t * dx))
            sample_y = float(start_xy[1] + (t * dy))
            reason = self.xy_violation_reason(
                sample_x,
                sample_y,
                boundary_tolerance_m=boundary_tolerance_m,
                boundary_margin_m=boundary_margin_m,
                keep_out_margin_m=keep_out_margin_m,
            )
            if reason is not None:
                return reason
        return None

    def goal_altitude_violation_reason(self, goal_z_m: float) -> Optional[str]:
        if (
            self.altitude_limits.min_z_m is not None
            and goal_z_m < self.altitude_limits.min_z_m
        ):
            return (
                f"goal z={goal_z_m:.2f} m is below min_z="
                f"{self.altitude_limits.min_z_m:.2f} m"
            )
        if (
            self.altitude_limits.max_z_m is not None
            and goal_z_m > self.altitude_limits.max_z_m
        ):
            return (
                f"goal z={goal_z_m:.2f} m is above max_z="
                f"{self.altitude_limits.max_z_m:.2f} m"
            )
        return None

    def runtime_ceiling_violation_reason(
        self,
        current_z_m: float,
        *,
        ceiling_tolerance_m: float = 0.0,
    ) -> Optional[str]:
        if self.altitude_limits.max_z_m is None:
            return None
        if current_z_m <= (self.altitude_limits.max_z_m + max(ceiling_tolerance_m, 0.0)):
            return None
        return (
            f"current z={current_z_m:.2f} m exceeds max_z="
            f"{self.altitude_limits.max_z_m:.2f} m"
        )


def _optional_float(value: object) -> Optional[float]:
    if value is None:
        return None
    return float(value)


def _parse_vertices(
    raw_vertices: object,
    *,
    region_name: str,
) -> tuple[tuple[float, float], ...]:
    if not isinstance(raw_vertices, list):
        raise ValueError(f"{region_name} must be a list of vertices")
    vertices: list[tuple[float, float]] = []
    for raw_vertex in raw_vertices:
        if not isinstance(raw_vertex, dict):
            raise ValueError(f"{region_name} vertices must be mappings")
        vertices.append(
            (
                float(raw_vertex["x_m"]),
                float(raw_vertex["y_m"]),
            )
        )
    if len(vertices) < 3:
        raise ValueError(f"{region_name} must contain at least 3 vertices")
    return tuple(vertices)
