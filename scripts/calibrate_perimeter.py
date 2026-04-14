#!/usr/bin/env python3
"""
Interactive studio perimeter calibration helper.

Capture the safe fly boundary and keep-out zones in the same local frame the
drone already uses for flight control. By default that is `/mavros/local_position/pose`.

Recommended workflow:
1. Launch the external-pose pipeline so `/mavros/local_position/pose` is live.
2. Place a calibration marker at each vertex of the safe outer boundary.
3. Place the marker at the four corners of the pillar keep-out square.
4. Let this script average each placement and write `perimeter.yaml`.

Example:
  python3 scripts/calibrate_perimeter.py \
    --topic /mavros/local_position/pose \
    --output ros2/src/drone_bringup/config/perimeter.yaml \
    --boundary-labels north_west north_east south_east south_west \
    --pillar-labels pillar_nw pillar_ne pillar_se pillar_sw \
    --z-min-m 0.35 \
    --z-max-m 1.90
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def quat_to_yaw_rad(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * ((q.w * q.z) + (q.x * q.y))
    cosy_cosp = 1.0 - 2.0 * ((q.y * q.y) + (q.z * q.z))
    return math.atan2(siny_cosp, cosy_cosp)


def round_float(value: float) -> float:
    return round(float(value), 4)


@dataclass
class PoseSample:
    label: str
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

    def to_yaml_dict(self) -> dict[str, float | str]:
        return {
            "name": self.label,
            "x_m": round_float(self.x_m),
            "y_m": round_float(self.y_m),
            "z_m": round_float(self.z_m),
            "yaw_rad": round_float(self.yaw_rad),
        }


class PoseCaptureNode(Node):
    def __init__(self, topic: str, best_effort: bool) -> None:
        super().__init__("perimeter_calibration_node")
        self.latest_pose: Optional[PoseStamped] = None
        qos = QoSProfile(
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if best_effort
                else ReliabilityPolicy.RELIABLE
            ),
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(PoseStamped, topic, self.pose_callback, qos)

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg


def wait_for_pose(node: PoseCaptureNode, timeout_s: float) -> PoseStamped:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.latest_pose is not None:
            return node.latest_pose
    raise TimeoutError("No pose data received before timeout")


def average_pose(
    node: PoseCaptureNode,
    sample_count: int,
    sample_interval_s: float,
) -> PoseSample:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    yaws: list[float] = []
    frame_id = ""

    while len(xs) < sample_count:
        rclpy.spin_once(node, timeout_sec=max(0.2, sample_interval_s))
        pose = node.latest_pose
        if pose is None:
            continue
        frame_id = pose.header.frame_id or frame_id or "map"
        xs.append(float(pose.pose.position.x))
        ys.append(float(pose.pose.position.y))
        zs.append(float(pose.pose.position.z))
        yaws.append(quat_to_yaw_rad(pose))
        time.sleep(sample_interval_s)

    yaw_x = float(np.mean(np.cos(np.asarray(yaws, dtype=float))))
    yaw_y = float(np.mean(np.sin(np.asarray(yaws, dtype=float))))
    mean_yaw = math.atan2(yaw_y, yaw_x)
    return PoseSample(
        label="",
        frame_id=frame_id or "map",
        x_m=float(np.mean(xs)),
        y_m=float(np.mean(ys)),
        z_m=float(np.mean(zs)),
        yaw_rad=mean_yaw,
    )


def capture_named_sample(
    node: PoseCaptureNode,
    label: str,
    sample_count: int,
    sample_interval_s: float,
) -> Optional[PoseSample]:
    while True:
        print(
            f"\nPlace the calibration marker at '{label}'. "
            "Press Enter to capture, or type 'skip' to omit it."
        )
        response = input("> ").strip().lower()
        if response == "skip":
            print(f"Skipping '{label}'.")
            return None

        sample = average_pose(node, sample_count, sample_interval_s)
        sample.label = label
        print(
            "Captured "
            f"{label}: x={sample.x_m:+.3f} m "
            f"y={sample.y_m:+.3f} m "
            f"z={sample.z_m:+.3f} m "
            f"yaw={math.degrees(sample.yaw_rad):+.1f} deg "
            f"frame={sample.frame_id}"
        )
        print("Accept this sample? Press Enter for yes, or type 'retry'.")
        confirm = input("> ").strip().lower()
        if confirm != "retry":
            return sample


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/mavros/local_position/pose",
        help="Pose topic to sample. Prefer /mavros/local_position/pose.",
    )
    parser.add_argument(
        "--output",
        default="ros2/src/drone_bringup/config/perimeter.yaml",
        help="Where to write the generated perimeter YAML.",
    )
    parser.add_argument(
        "--boundary-labels",
        nargs="+",
        default=["north_west", "north_east", "south_east", "south_west"],
        help="Ordered safe-boundary vertices to capture.",
    )
    parser.add_argument(
        "--pillar-labels",
        nargs="*",
        default=["pillar_nw", "pillar_ne", "pillar_se", "pillar_sw"],
        help="Ordered vertices of the pillar keep-out polygon.",
    )
    parser.add_argument(
        "--pillar-name",
        default="studio_pillar",
        help="Name of the pillar keep-out zone in the output YAML.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=25,
        help="How many pose samples to average for each capture.",
    )
    parser.add_argument(
        "--sample-interval-s",
        type=float,
        default=0.04,
        help="Delay between averaged pose samples.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=10.0,
        help="How long to wait for the first pose sample.",
    )
    parser.add_argument(
        "--frame-id",
        default="",
        help="Optional output frame override. Defaults to the sampled pose frame.",
    )
    parser.add_argument(
        "--z-min-m",
        type=float,
        default=0.35,
        help="Minimum safe flight altitude to store in the YAML.",
    )
    parser.add_argument(
        "--z-max-m",
        type=float,
        default=1.90,
        help="Maximum safe flight altitude to store in the YAML.",
    )
    parser.add_argument(
        "--best-effort",
        action="store_true",
        help="Use BEST_EFFORT QoS for the pose topic subscription.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    rclpy.init()
    node = PoseCaptureNode(args.topic, args.best_effort)

    try:
        print(f"Waiting for pose data on {args.topic} ...")
        initial_pose = wait_for_pose(node, timeout_s=args.timeout_s)
        sampled_frame = initial_pose.header.frame_id or "map"
        frame_id = args.frame_id.strip() or sampled_frame
        print(f"Pose stream is live. Using frame_id='{frame_id}'.")

        boundary_samples: list[PoseSample] = []
        for label in args.boundary_labels:
            sample = capture_named_sample(
                node,
                label=label,
                sample_count=max(args.samples, 1),
                sample_interval_s=max(args.sample_interval_s, 0.0),
            )
            if sample is not None:
                boundary_samples.append(sample)

        if len(boundary_samples) < 3:
            raise RuntimeError("Need at least 3 boundary samples to define a perimeter")

        pillar_samples: list[PoseSample] = []
        for label in args.pillar_labels:
            sample = capture_named_sample(
                node,
                label=label,
                sample_count=max(args.samples, 1),
                sample_interval_s=max(args.sample_interval_s, 0.0),
            )
            if sample is not None:
                pillar_samples.append(sample)

        output: dict[str, object] = {
            "version": 1,
            "frame_id": frame_id,
            "calibration_source_topic": args.topic,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "capture_semantics": {
                "boundary_polygon_xy_m": (
                    "Safe inner boundary to enforce directly, not the physical wall line"
                ),
                "keep_out_polygons_xy_m": (
                    "Blocked XY polygons the drone must not enter"
                ),
            },
            "altitude_limits_m": {
                "min_z_m": round_float(args.z_min_m),
                "max_z_m": round_float(args.z_max_m),
            },
            "boundary_polygon_xy_m": [
                {
                    "name": sample.label,
                    "x_m": round_float(sample.x_m),
                    "y_m": round_float(sample.y_m),
                }
                for sample in boundary_samples
            ],
            "keep_out_polygons_xy_m": [],
            "recorded_samples": {
                "boundary_vertices": [sample.to_yaml_dict() for sample in boundary_samples],
                "pillar_vertices": [sample.to_yaml_dict() for sample in pillar_samples],
            },
            "recommended_policy": {
                "reject_goals_outside_boundary": True,
                "block_motion_toward_violation": True,
                "hold_zero_velocity_on_violation": True,
            },
        }

        if pillar_samples:
            output["keep_out_polygons_xy_m"].append(
                {
                    "name": args.pillar_name,
                    "vertices_xy_m": [
                        {
                            "name": sample.label,
                            "x_m": round_float(sample.x_m),
                            "y_m": round_float(sample.y_m),
                        }
                        for sample in pillar_samples
                    ],
                }
            )
            print(f"\nRecorded {len(pillar_samples)} pillar polygon vertices.")

        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(output, stream, sort_keys=False)

        print(f"\nWrote perimeter config to {output_path}")
        return 0
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"\nCalibration failed: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
