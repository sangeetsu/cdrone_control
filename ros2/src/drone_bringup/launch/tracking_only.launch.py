import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _vision_launch_path(package_name: str, leaf_name: str) -> str:
    return os.path.join(get_package_share_directory(package_name), "launch", leaf_name)


def generate_launch_description():
    vision_share = get_package_share_directory("drone_vision_pkg")
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("source_mode", default_value="direct"),
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
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=os.path.join(
                    vision_share,
                    "config",
                    "tracking_only.yaml",
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _vision_launch_path("drone_vision_pkg", "tracking_only.launch.py")
                ),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
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
                    "reid_histogram_bins": LaunchConfiguration(
                        "reid_histogram_bins"
                    ),
                    "reid_distance_threshold": LaunchConfiguration(
                        "reid_distance_threshold"
                    ),
                    "reid_hit_counter_max": LaunchConfiguration(
                        "reid_hit_counter_max"
                    ),
                    "enable_motion_estimator": LaunchConfiguration(
                        "enable_motion_estimator"
                    ),
                    "publish_track_hold_s": LaunchConfiguration(
                        "publish_track_hold_s"
                    ),
                    "publish_track_hold_max_extrapolation_m": LaunchConfiguration(
                        "publish_track_hold_max_extrapolation_m"
                    ),
                    "tracker_config_file": LaunchConfiguration("tracker_config_file"),
                }.items(),
            ),
        ]
    )
