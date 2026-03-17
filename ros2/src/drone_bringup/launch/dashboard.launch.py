from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone_id = LaunchConfiguration("drone_id")
    dashboard_rate_hz = LaunchConfiguration("dashboard_rate_hz")
    log_level = LaunchConfiguration("log_level")

    dashboard_tui_node = Node(
        package="drone_monitor_pkg",
        executable="dashboard_tui_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "dashboard_rate_hz": dashboard_rate_hz,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )
    return [dashboard_tui_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("dashboard_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
