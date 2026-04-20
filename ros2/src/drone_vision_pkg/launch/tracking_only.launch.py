from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_share = Path(__file__).resolve().parents[1]
    default_tracker_config = str(vision_share / "config" / "tracking_only.yaml")

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
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("source_mode", default_value="direct"),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=default_tracker_config,
            ),
            DeclareLaunchArgument(
                "pose_topic", default_value="/vrpn_mocap/RigidBody3/pose"
            ),
            DeclareLaunchArgument(
                "publish_world_track_compare", default_value="true"
            ),
            DeclareLaunchArgument(
                "compare_pose_topic", default_value="/vrpn_mocap/RigidBody2/pose"
            ),
            DeclareLaunchArgument("world_track_compare_topic", default_value=""),
            DeclareLaunchArgument("experiment_tag", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            tracker_node,
        ]
    )
