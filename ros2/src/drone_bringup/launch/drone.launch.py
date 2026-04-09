import os
from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone_bringup_pkg_share = get_package_share_directory("drone_bringup")
    # Parameters for Mavros.
    mavros_params = os.path.join(drone_bringup_pkg_share, "config", "apm_params.yaml")
    mavros_config = os.path.join(drone_bringup_pkg_share, "config", "apm_config.yaml")
    mavros_plugins = os.path.join(
        drone_bringup_pkg_share, "config", "apm_pluginlists.yaml"
    )
    with open(mavros_params, "r", encoding="utf-8") as stream:
        mavros_param_dict = yaml.safe_load(stream) or {}
    mavros_node_params = (
        mavros_param_dict.get("mavros_node", {}).get("ros__parameters", {})
    )

    # Launch arguments
    log_level = LaunchConfiguration("log_level")
    mavros_namespace = LaunchConfiguration("mavros_namespace")

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        namespace=mavros_namespace,
        output="screen",
        parameters=[mavros_node_params, mavros_config, mavros_plugins],
        arguments=["--ros-args", "--log-level", log_level],
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
