"""ZED local-pose square demo built on the proven hover launch path."""

import os

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    control_share = get_package_share_directory("drone_control_pkg")
    defaults = load_drone_launch_defaults()
    forwarded = (
        "log_level", "localization_source", "enable_flight", "drone_id",
        "mavros_namespace", "zed_mount_config", "takeoff_altitude_m",
        "takeoff_rate_m_s", "hover_duration_s", "max_horizontal_excursion_m",
        "initial_climb_enabled", "square_edge_m", "square_speed_m_s",
        "square_setpoint_lead_m", "square_waypoint_tolerance_m",
        "square_waypoint_dwell_s", "square_waypoint_timeout_s",
        "square_max_excursion_m", "square_max_path_error_m",
        "zed_takeoff_setpoint_lead_m", "zed_takeoff_no_lift_timeout_s",
    )
    hover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "position_hover_demo.launch.py")),
        launch_arguments={
            **{name: LaunchConfiguration(name) for name in forwarded},
            "mission_mode": "square",
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("localization_source", default_value="zed", choices=["zed"]),
        DeclareLaunchArgument("enable_flight", default_value="false"),
        DeclareLaunchArgument("drone_id", default_value=default_arg(defaults, "drone_id", "")),
        DeclareLaunchArgument("mavros_namespace", default_value=default_arg(
            defaults, "mavros_namespace", "mavros")),
        DeclareLaunchArgument("zed_mount_config", default_value=os.path.join(
            control_share, "config", "zed_hover_mount.yaml")),
        DeclareLaunchArgument("takeoff_altitude_m", default_value="1.0"),
        DeclareLaunchArgument("takeoff_rate_m_s", default_value="0.3"),
        DeclareLaunchArgument("hover_duration_s", default_value="20.0"),
        DeclareLaunchArgument("max_horizontal_excursion_m", default_value="0.5"),
        DeclareLaunchArgument("initial_climb_enabled", default_value="false"),
        DeclareLaunchArgument("square_edge_m", default_value="1.0"),
        DeclareLaunchArgument("square_speed_m_s", default_value="0.2"),
        DeclareLaunchArgument("square_setpoint_lead_m", default_value="0.3"),
        DeclareLaunchArgument("square_waypoint_tolerance_m", default_value="0.15"),
        DeclareLaunchArgument("square_waypoint_dwell_s", default_value="1.0"),
        DeclareLaunchArgument("square_waypoint_timeout_s", default_value="20.0"),
        DeclareLaunchArgument("square_max_excursion_m", default_value="1.8"),
        DeclareLaunchArgument("square_max_path_error_m", default_value="0.35"),
        DeclareLaunchArgument("zed_takeoff_setpoint_lead_m", default_value="0.6"),
        DeclareLaunchArgument("zed_takeoff_no_lift_timeout_s", default_value="8.0"),
        hover_launch,
    ])
