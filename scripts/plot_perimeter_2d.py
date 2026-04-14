#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import yaml


def load_config(path: Path) -> dict:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"expected mapping in {path}")
    return loaded


def closed_xy(vertices: list[tuple[float, float]]) -> tuple[list[float], list[float]]:
    points = vertices + [vertices[0]]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return xs, ys


def annotate_vertices(ax, vertices, *, color: str, prefix: str) -> None:
    for idx, (x_m, y_m) in enumerate(vertices, start=1):
        ax.scatter([x_m], [y_m], color=color, s=28, zorder=5)
        ax.annotate(
            f"{prefix}{idx}\n({x_m:.2f}, {y_m:.2f})",
            (x_m, y_m),
            textcoords="offset points",
            xytext=(8, 6),
            fontsize=8,
            color=color,
        )


def polygon_centroid(vertices: list[tuple[float, float]]) -> tuple[float, float]:
    area_twice = 0.0
    centroid_x = 0.0
    centroid_y = 0.0
    for idx in range(len(vertices)):
        x1, y1 = vertices[idx]
        x2, y2 = vertices[(idx + 1) % len(vertices)]
        cross = (x1 * y2) - (x2 * y1)
        area_twice += cross
        centroid_x += (x1 + x2) * cross
        centroid_y += (y1 + y2) * cross
    if math.isclose(area_twice, 0.0, abs_tol=1e-9):
        return (
            sum(vertex[0] for vertex in vertices) / len(vertices),
            sum(vertex[1] for vertex in vertices) / len(vertices),
        )
    return (
        centroid_x / (3.0 * area_twice),
        centroid_y / (3.0 * area_twice),
    )


def max_radius_from_center(
    center_xy: tuple[float, float],
    vertices: list[tuple[float, float]],
) -> float:
    max_radius_m = 0.0
    for idx in range(len(vertices)):
        start_xy = vertices[idx]
        end_xy = vertices[(idx + 1) % len(vertices)]
        for sample_idx in range(51):
            t = sample_idx / 50.0
            sample_xy = (
                start_xy[0] + ((end_xy[0] - start_xy[0]) * t),
                start_xy[1] + ((end_xy[1] - start_xy[1]) * t),
            )
            max_radius_m = max(
                max_radius_m,
                math.hypot(
                    sample_xy[0] - center_xy[0],
                    sample_xy[1] - center_xy[1],
                ),
            )
    return max_radius_m


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a 2D perimeter map.")
    parser.add_argument(
        "--config",
        default="ros2/src/drone_bringup/config/drone_studio_perimeter.yaml",
        help="Perimeter YAML to visualize.",
    )
    parser.add_argument(
        "--output",
        default="docs/generated/drone_studio_perimeter_2d.svg",
        help="Output SVG path.",
    )
    args = parser.parse_args()

    config_path = Path(args.config).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    config = load_config(config_path)

    boundary = [
        (float(item["x_m"]), float(item["y_m"]))
        for item in config["boundary_polygon_xy_m"]
    ]
    keep_out = [
        (
            float(item["x_m"]),
            float(item["y_m"]),
        )
        for item in config["keep_out_polygons_xy_m"][0]["vertices_xy_m"]
    ]

    keep_out_clearance_m = 1.2
    circle_center = polygon_centroid(keep_out)
    circle_radius_m = max_radius_from_center(circle_center, keep_out) + keep_out_clearance_m

    safe_short_goto = (-11.5, -2.5)
    circle_stage_goto = (circle_center[0], circle_center[1] - circle_radius_m)
    retired_edge_goal = (4.164525, 0.1813)

    fig, ax = plt.subplots(figsize=(11, 8.5))

    bx, by = closed_xy(boundary)
    px, py = closed_xy(keep_out)
    boundary_patch = ax.fill(
        bx,
        by,
        facecolor="#d8f3dc",
        edgecolor="#1b4332",
        linewidth=2.2,
        alpha=0.55,
        label="Studio boundary",
        zorder=1,
    )
    pillar_patch = ax.fill(
        px,
        py,
        facecolor="#ffccd5",
        edgecolor="#c1121f",
        linewidth=2.0,
        alpha=0.75,
        label="Pillar keep-out",
        zorder=2,
    )
    del boundary_patch, pillar_patch

    circle = plt.Circle(
        circle_center,
        circle_radius_m,
        edgecolor="#005f73",
        facecolor="none",
        linewidth=2.0,
        linestyle="--",
        label="Circle demo orbit around pillar",
        zorder=3,
    )
    ax.add_patch(circle)

    annotate_vertices(ax, boundary, color="#1b4332", prefix="B")
    annotate_vertices(ax, keep_out, color="#c1121f", prefix="P")

    ax.scatter(
        [safe_short_goto[0], circle_stage_goto[0], retired_edge_goal[0], circle_center[0]],
        [safe_short_goto[1], circle_stage_goto[1], retired_edge_goal[1], circle_center[1]],
        color=["#2a9d8f", "#2a9d8f", "#d62828", "#005f73"],
        marker="o",
        s=[64, 64, 72, 64],
        zorder=6,
    )
    ax.annotate(
        "Phase 2 short goto\n(-11.5, -2.5)",
        safe_short_goto,
        textcoords="offset points",
        xytext=(10, -18),
        fontsize=9,
        color="#2a9d8f",
    )
    ax.annotate(
        f"Phase 3 circle staging goto\n({circle_stage_goto[0]:.2f}, {circle_stage_goto[1]:.2f})",
        circle_stage_goto,
        textcoords="offset points",
        xytext=(10, 10),
        fontsize=9,
        color="#2a9d8f",
    )
    ax.annotate(
        "Retired edge goal\n(4.16, 0.18)",
        retired_edge_goal,
        textcoords="offset points",
        xytext=(12, -28),
        fontsize=9,
        color="#d62828",
    )
    ax.annotate(
        f"Circle center from pillar\n({circle_center[0]:.2f}, {circle_center[1]:.2f})",
        circle_center,
        textcoords="offset points",
        xytext=(10, 12),
        fontsize=9,
        color="#005f73",
    )

    ax.axhline(0.0, color="#6c757d", linewidth=1.0, alpha=0.9)
    ax.axvline(0.0, color="#6c757d", linewidth=1.0, alpha=0.9)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (m) in map frame")
    ax.set_ylabel("y (m) in map frame")
    ax.set_title("Drone Studio Perimeter and Test Coordinates")
    ax.grid(True, which="major", color="#adb5bd", alpha=0.5, linestyle=":")
    ax.minorticks_on()
    ax.grid(True, which="minor", color="#dee2e6", alpha=0.35, linestyle=":")

    all_x = [p[0] for p in boundary] + [p[0] for p in keep_out]
    all_y = [p[1] for p in boundary] + [p[1] for p in keep_out]
    margin_m = 1.5
    ax.set_xlim(min(all_x) - margin_m, max(all_x) + margin_m)
    ax.set_ylim(min(all_y) - margin_m, max(all_y) + margin_m)

    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(output_path, format="svg")
    print(output_path)


if __name__ == "__main__":
    main()
