#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = REPO_ROOT / "docs" / "generated"


def draw_box(ax, xy, width, height, label, *, facecolor, edgecolor="#253238"):
    patch = Rectangle(
        xy,
        width,
        height,
        linewidth=1.8,
        edgecolor=edgecolor,
        facecolor=facecolor,
        zorder=2,
    )
    ax.add_patch(patch)
    ax.text(
        xy[0] + width / 2.0,
        xy[1] + height / 2.0,
        label,
        ha="center",
        va="center",
        fontsize=10,
        zorder=3,
    )
    return patch


def draw_arrow(ax, start, end, label, *, color="#334e68", rad=0.0):
    arrow = FancyArrowPatch(
        start,
        end,
        arrowstyle="-|>",
        mutation_scale=14,
        linewidth=1.6,
        color=color,
        connectionstyle=f"arc3,rad={rad}",
        zorder=1,
    )
    ax.add_patch(arrow)
    mid = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
    ax.text(
        mid[0],
        mid[1] + 0.12,
        label,
        ha="center",
        va="bottom",
        fontsize=8,
        color=color,
        zorder=4,
    )


def write_architecture_svg(output_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(12, 7))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 7)
    ax.axis("off")

    draw_box(
        ax,
        (0.55, 5.25),
        2.1,
        0.9,
        "OptiTrack pose\n(default map frame)",
        facecolor="#d9f0ff",
    )
    draw_box(
        ax,
        (0.55, 3.65),
        2.1,
        0.9,
        "Isaac VSLAM\n(optional pose)",
        facecolor="#e9e2ff",
    )
    draw_box(
        ax,
        (0.55, 1.95),
        2.1,
        0.9,
        "D455 + YOLO\nRealSense tracker",
        facecolor="#e5f8df",
    )
    draw_box(
        ax,
        (3.85, 2.65),
        2.35,
        1.25,
        "Target map\nstable IDs + prediction",
        facecolor="#fff4cc",
    )
    draw_box(
        ax,
        (7.25, 4.55),
        2.15,
        0.95,
        "RViz markers\npath + uncertainty",
        facecolor="#ffe5d9",
    )
    draw_box(
        ax,
        (7.25, 2.45),
        2.15,
        0.95,
        "Milestone 4\ncautious control gate",
        facecolor="#e5f8f3",
    )
    draw_box(
        ax,
        (10.1, 2.45),
        1.45,
        0.95,
        "MAVROS\ncmd_vel",
        facecolor="#f2f4f8",
    )

    draw_arrow(ax, (2.65, 5.7), (3.85, 3.55), "/pose", rad=-0.12)
    draw_arrow(ax, (2.65, 4.1), (3.85, 3.35), "/pose", rad=0.08)
    draw_arrow(ax, (2.65, 2.4), (3.85, 3.0), "/perception/world_tracks")
    draw_arrow(ax, (6.2, 3.38), (7.25, 5.02), "/target_map/markers", rad=0.18)
    draw_arrow(ax, (6.2, 3.0), (7.25, 2.92), "/target_map/tracks")
    draw_arrow(ax, (9.4, 2.92), (10.1, 2.92), "/control/cmd_vel_body")

    ax.text(
        6.0,
        0.85,
        "Milestone 4 keeps raw detector output visible, but the controller consumes "
        "the target-map topic with prediction age and uncertainty gates.",
        ha="center",
        va="center",
        fontsize=10,
        color="#253238",
    )

    fig.tight_layout()
    fig.savefig(output_path, format="svg")
    plt.close(fig)


def write_prediction_path_svg(output_path: Path) -> None:
    observed_t = np.array([0.0, 0.5, 1.0, 1.5])
    observed_x = np.array([0.0, 0.45, 0.95, 1.45])
    observed_y = np.array([0.0, 0.08, 0.16, 0.18])
    predicted_t = np.array([2.0, 2.5, 3.0])
    predicted_x = np.array([1.95, 2.45, 2.95])
    predicted_y = np.array([0.2, 0.22, 0.24])
    confidence = np.array([0.86, 0.74, 0.62])
    uncertainty = np.array([0.28, 0.46, 0.64])

    fig, ax = plt.subplots(figsize=(11, 7))
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-0.5, 3.7)
    ax.set_ylim(-0.75, 1.3)
    ax.grid(True, linestyle=":", alpha=0.35)
    ax.set_xlabel("x (m) in map frame")
    ax.set_ylabel("y (m) in map frame")
    ax.set_title("Milestone 4 Target Memory: Observed Track To Short Prediction")

    ax.plot(observed_x, observed_y, color="#1b7f3a", linewidth=2.3, label="observed")
    ax.scatter(observed_x, observed_y, color="#1b7f3a", s=60, zorder=4)
    ax.plot(
        np.r_[observed_x[-1], predicted_x],
        np.r_[observed_y[-1], predicted_y],
        color="#f06d06",
        linewidth=2.1,
        linestyle="--",
        label="predicted while out of frame",
    )
    ax.scatter(predicted_x, predicted_y, color="#f06d06", s=60, zorder=4)

    for x_m, y_m, radius_m, conf in zip(predicted_x, predicted_y, uncertainty, confidence):
        circle = Circle(
            (x_m, y_m),
            radius_m,
            edgecolor="#f06d06",
            facecolor="#f06d06",
            alpha=0.13,
            linewidth=1.6,
        )
        ax.add_patch(circle)
        ax.text(
            x_m,
            y_m + radius_m + 0.08,
            f"conf {conf:.2f}\nunc {radius_m:.2f} m",
            ha="center",
            va="bottom",
            fontsize=8,
            color="#8a3b00",
        )

    ax.annotate(
        "last detector observation",
        (observed_x[-1], observed_y[-1]),
        xytext=(-25, 42),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": "#334e68"},
        fontsize=9,
        color="#334e68",
    )
    ax.annotate(
        "control gate accepts only\nshort, low-uncertainty predictions",
        (predicted_x[1], predicted_y[1]),
        xytext=(48, -54),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": "#334e68"},
        fontsize=9,
        color="#334e68",
    )
    ax.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(output_path, format="svg")
    plt.close(fig)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    architecture_path = OUTPUT_DIR / "milestone4_target_memory_architecture.svg"
    prediction_path = OUTPUT_DIR / "milestone4_prediction_path.svg"
    write_architecture_svg(architecture_path)
    write_prediction_path_svg(prediction_path)
    print(architecture_path)
    print(prediction_path)


if __name__ == "__main__":
    main()
