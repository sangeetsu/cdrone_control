import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    source_mode = LaunchConfiguration("source_mode").perform(context)
    vio_params = LaunchConfiguration("vio_params").perform(context)
    bridge_params = LaunchConfiguration("bridge_params").perform(context)

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    stereo_vio_node = Node(
        package="drone_vision_pkg",
        executable="stereo_vio_node",
        output="screen",
        parameters=[vio_params, {"source_mode": source_mode}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    px4_vision_bridge_node = Node(
        package="drone_control_pkg",
        executable="px4_vision_bridge_node",
        output="screen",
        parameters=[bridge_params],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [drone_launch, stereo_vio_node, px4_vision_bridge_node]


def generate_launch_description():
    vision_share = get_package_share_directory("drone_vision_pkg")
    control_share = get_package_share_directory("drone_control_pkg")
    default_vio_params = os.path.join(vision_share, "config", "stereo_vio.yaml")
    default_bridge_params = os.path.join(
        control_share, "config", "px4_vision_bridge.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("source_mode", default_value="csi"),
            DeclareLaunchArgument("vio_params", default_value=default_vio_params),
            DeclareLaunchArgument(
                "bridge_params", default_value=default_bridge_params
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
