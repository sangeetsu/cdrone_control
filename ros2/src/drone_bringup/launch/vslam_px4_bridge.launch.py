import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    drone_id = LaunchConfiguration("drone_id")

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    vio_bridge_node = Node(
        package="drone_control_pkg",
        executable="vio_bridge_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "input_timeout_s": ParameterValue(
                    LaunchConfiguration("input_timeout_s"), value_type=float
                ),
                "publish_companion_status": ParameterValue(
                    LaunchConfiguration("publish_companion_status"), value_type=bool
                ),
                "input_pose_topic": LaunchConfiguration("input_pose_topic"),
                "output_pose_topic": LaunchConfiguration("output_pose_topic"),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [drone_launch, vio_bridge_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            DeclareLaunchArgument(
                "input_pose_topic",
                default_value="/visual_slam/tracking/vo_pose",
            ),
            DeclareLaunchArgument("output_pose_topic", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("input_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("publish_companion_status", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
