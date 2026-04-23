from pathlib import Path

from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_share = Path(__file__).resolve().parents[1]
    default_tracker_config = str(vision_share / "config" / "tracking_only.yaml")
    defaults = load_drone_launch_defaults()

    tracker_node = Node(
        package="drone_vision_pkg",
        executable="realsense_tracker_node",
        output="screen",
        parameters=[
            LaunchConfiguration("tracker_config_file"),
            {
                "drone_id": LaunchConfiguration("drone_id"),
                "source_mode": LaunchConfiguration("source_mode"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "publish_world_track_compare": LaunchConfiguration(
                    "publish_world_track_compare"
                ),
                "compare_pose_topic": LaunchConfiguration("compare_pose_topic"),
                "world_track_compare_topic": LaunchConfiguration(
                    "world_track_compare_topic"
                ),
                "experiment_tag": LaunchConfiguration("experiment_tag"),
                "model_path": LaunchConfiguration("model_path"),
                "enable_reid": LaunchConfiguration("enable_reid"),
                "reid_histogram_bins": LaunchConfiguration("reid_histogram_bins"),
                "reid_distance_threshold": LaunchConfiguration(
                    "reid_distance_threshold"
                ),
                "reid_hit_counter_max": LaunchConfiguration("reid_hit_counter_max"),
                "enable_motion_estimator": LaunchConfiguration(
                    "enable_motion_estimator"
                ),
                "publish_track_hold_s": LaunchConfiguration("publish_track_hold_s"),
                "publish_track_hold_max_extrapolation_m": LaunchConfiguration(
                    "publish_track_hold_max_extrapolation_m"
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "drone_id",
                default_value=default_arg(defaults, "drone_id", ""),
            ),
            DeclareLaunchArgument("source_mode", default_value="direct"),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=default_tracker_config,
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value=default_arg(defaults, "ownship_pose_topic", ""),
            ),
            DeclareLaunchArgument(
                "publish_world_track_compare", default_value="true"
            ),
            DeclareLaunchArgument(
                "compare_pose_topic",
                default_value=default_arg(defaults, "compare_pose_topic", ""),
            ),
            DeclareLaunchArgument("world_track_compare_topic", default_value=""),
            DeclareLaunchArgument("experiment_tag", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("enable_reid", default_value="false"),
            DeclareLaunchArgument("reid_histogram_bins", default_value="32"),
            DeclareLaunchArgument(
                "reid_distance_threshold",
                default_value="0.20",
            ),
            DeclareLaunchArgument("reid_hit_counter_max", default_value="30"),
            DeclareLaunchArgument(
                "enable_motion_estimator",
                default_value="false",
            ),
            DeclareLaunchArgument("publish_track_hold_s", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_track_hold_max_extrapolation_m",
                default_value="0.25",
            ),
            tracker_node,
        ]
    )
