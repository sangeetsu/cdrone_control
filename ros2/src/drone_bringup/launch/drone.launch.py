import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    mavros_params = os.path.join(bringup_share, "config", "px4_params.yaml")
    mavros_config = os.path.join(bringup_share, "config", "px4_config.yaml")
    mavros_plugins = os.path.join(bringup_share, "config", "px4_pluginlists.yaml")

    with open(mavros_params, "r", encoding="utf-8") as stream:
        mavros_param_dict = yaml.safe_load(stream) or {}
    mavros_node_params = (
        mavros_param_dict.get("mavros_node", {}).get("ros__parameters", {})
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=LaunchConfiguration("mavros_namespace"),
        output="screen",
        parameters=[mavros_node_params, mavros_config, mavros_plugins],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return [mavros_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            OpaqueFunction(function=launch_setup),
        ]
    )
