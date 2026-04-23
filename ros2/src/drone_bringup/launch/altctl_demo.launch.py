import os

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    drone_id = LaunchConfiguration("drone_id")
    mavros_namespace = LaunchConfiguration("mavros_namespace")

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    demo_node = Node(
        package="drone_control_pkg",
        executable="altctl_demo_sequence_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"),
                    value_type=float,
                ),
                "target_altitude_m": ParameterValue(
                    LaunchConfiguration("target_altitude_m"),
                    value_type=float,
                ),
                "forward_distance_m": ParameterValue(
                    LaunchConfiguration("forward_distance_m"),
                    value_type=float,
                ),
                "hover_after_takeoff_s": ParameterValue(
                    LaunchConfiguration("hover_after_takeoff_s"),
                    value_type=float,
                ),
                "hover_after_translate_s": ParameterValue(
                    LaunchConfiguration("hover_after_translate_s"),
                    value_type=float,
                ),
                "arm_zero_throttle_hold_s": ParameterValue(
                    LaunchConfiguration("arm_zero_throttle_hold_s"),
                    value_type=float,
                ),
                "takeoff_throttle_delta": ParameterValue(
                    LaunchConfiguration("takeoff_throttle_delta"),
                    value_type=float,
                ),
                "hover_throttle_center": ParameterValue(
                    LaunchConfiguration("hover_throttle_center"),
                    value_type=float,
                ),
                "forward_stick_cmd": ParameterValue(
                    LaunchConfiguration("forward_stick_cmd"),
                    value_type=float,
                ),
                "altitude_tolerance_m": ParameterValue(
                    LaunchConfiguration("altitude_tolerance_m"),
                    value_type=float,
                ),
                "position_tolerance_m": ParameterValue(
                    LaunchConfiguration("position_tolerance_m"),
                    value_type=float,
                ),
                "stage_timeout_s": ParameterValue(
                    LaunchConfiguration("stage_timeout_s"),
                    value_type=float,
                ),
                "land_detect_altitude_m": ParameterValue(
                    LaunchConfiguration("land_detect_altitude_m"),
                    value_type=float,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [drone_launch, demo_node]


def generate_launch_description():
    defaults = load_drone_launch_defaults()
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
            DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("target_altitude_m", default_value="0.7"),
            DeclareLaunchArgument("forward_distance_m", default_value="1.2"),
            DeclareLaunchArgument("hover_after_takeoff_s", default_value="2.0"),
            DeclareLaunchArgument("hover_after_translate_s", default_value="2.0"),
            DeclareLaunchArgument("arm_zero_throttle_hold_s", default_value="1.5"),
            DeclareLaunchArgument("takeoff_throttle_delta", default_value="180.0"),
            DeclareLaunchArgument("hover_throttle_center", default_value="500.0"),
            DeclareLaunchArgument("forward_stick_cmd", default_value="220.0"),
            DeclareLaunchArgument("altitude_tolerance_m", default_value="0.1"),
            DeclareLaunchArgument("position_tolerance_m", default_value="0.1"),
            DeclareLaunchArgument("stage_timeout_s", default_value="20.0"),
            DeclareLaunchArgument("land_detect_altitude_m", default_value="0.15"),
            OpaqueFunction(function=launch_setup),
        ]
    )
