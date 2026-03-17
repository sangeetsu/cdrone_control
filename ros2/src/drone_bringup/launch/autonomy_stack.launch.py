import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    source_mode = LaunchConfiguration("source_mode").perform(context)
    scenario_name = LaunchConfiguration("scenario").perform(context)
    autonomy_params = LaunchConfiguration("autonomy_params").perform(context)
    log_level = LaunchConfiguration("log_level")
    drone_id = LaunchConfiguration("drone_id")
    hostname = LaunchConfiguration("hostname")
    self_ip = LaunchConfiguration("self_ip")
    mavros_namespace = LaunchConfiguration("mavros_namespace")

    bringup_share = get_package_share_directory("drone_bringup")
    behavior_share = get_package_share_directory("drone_behavior_pkg")
    scenario_file = os.path.join(
        behavior_share, "config", "scenarios", f"{scenario_name}.yaml"
    )
    if not os.path.exists(scenario_file):
        scenario_file = scenario_name

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    common_params = [
        autonomy_params,
        {
            "source_mode": source_mode,
            "scenario_file": scenario_file,
            "drone_id": drone_id,
            "hostname": hostname,
            "self_ip": self_ip,
            "mavros_namespace": mavros_namespace,
        },
    ]

    stereo_tracker_node = Node(
        package="drone_vision_pkg",
        executable="stereo_tracker_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    vio_bridge_node = Node(
        package="drone_control_pkg",
        executable="vio_bridge_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    engagement_manager_node = Node(
        package="drone_behavior_pkg",
        executable="engagement_manager_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    health_monitor_node = Node(
        package="drone_behavior_pkg",
        executable="health_monitor_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    mavros_velocity_node = Node(
        package="drone_control_pkg",
        executable="mavros_velocity_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    light_controller_node = Node(
        package="drone_light_pkg",
        executable="light_controller_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )
    telemetry_aggregator_node = Node(
        package="drone_monitor_pkg",
        executable="telemetry_aggregator_node",
        output="screen",
        parameters=common_params,
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [
        drone_launch,
        stereo_tracker_node,
        vio_bridge_node,
        engagement_manager_node,
        health_monitor_node,
        mavros_velocity_node,
        light_controller_node,
        telemetry_aggregator_node,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    default_params = os.path.join(bringup_share, "config", "autonomy_params.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("source_mode", default_value="csi"),
            DeclareLaunchArgument("scenario", default_value="track_follow_v1"),
            DeclareLaunchArgument("autonomy_params", default_value=default_params),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("hostname", default_value="cdrone-orin"),
            DeclareLaunchArgument("self_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
