import os

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    defaults = load_drone_launch_defaults()
    external_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "external_pose_px4_bridge.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "drone_id": LaunchConfiguration("drone_id"),
            "mavros_namespace": LaunchConfiguration("mavros_namespace"),
            "pose_source": "realsense_pose",
            "source_pose_topic": LaunchConfiguration("input_pose_topic"),
            "output_pose_topic": LaunchConfiguration("output_pose_topic"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "input_timeout_s": LaunchConfiguration("input_timeout_s"),
            "adapter_timeout_s": LaunchConfiguration("input_timeout_s"),
            "publish_companion_status": LaunchConfiguration("publish_companion_status"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "drone_id",
                default_value=default_arg(defaults, "drone_id", ""),
            ),
            DeclareLaunchArgument(
                "mavros_namespace",
                default_value=default_arg(defaults, "mavros_namespace", "mavros"),
            ),
            DeclareLaunchArgument(
                "input_pose_topic",
                default_value="/visual_slam/tracking/vo_pose",
            ),
            DeclareLaunchArgument("output_pose_topic", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("input_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("publish_companion_status", default_value="true"),
            external_pose_launch,
        ]
    )
