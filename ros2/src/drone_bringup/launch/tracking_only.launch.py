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
            DeclareLaunchArgument("model_path", default_value=""),
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
                    "model_path": LaunchConfiguration("model_path"),
                    "tracker_config_file": LaunchConfiguration("tracker_config_file"),
                }.items(),
            ),
        ]
    )
