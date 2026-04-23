import os

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import (
    default_arg as _default_arg,
    load_drone_launch_defaults,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_optitrack_defaults() -> dict[str, object]:
    return load_drone_launch_defaults()


def launch_setup(context, *args, **kwargs):
    del context, args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    drone_id = LaunchConfiguration("drone_id")
    mavros_namespace = LaunchConfiguration("mavros_namespace")

    pose_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "external_pose_px4_bridge.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "drone_id": drone_id,
            "mavros_namespace": mavros_namespace,
            "pose_source": LaunchConfiguration("pose_source"),
            "source_pose_topic": LaunchConfiguration("source_pose_topic"),
            "enable_reference_setup": "true",
            "global_origin_latitude_deg": LaunchConfiguration(
                "global_origin_latitude_deg"
            ),
            "global_origin_longitude_deg": LaunchConfiguration(
                "global_origin_longitude_deg"
            ),
            "global_origin_altitude_m": LaunchConfiguration(
                "global_origin_altitude_m"
            ),
            "home_position_x_m": LaunchConfiguration("home_position_x_m"),
            "home_position_y_m": LaunchConfiguration("home_position_y_m"),
            "home_position_z_m": LaunchConfiguration("home_position_z_m"),
            "home_approach_z_m": LaunchConfiguration("home_approach_z_m"),
            "reference_retry_period_s": LaunchConfiguration("reference_retry_period_s"),
            "publish_rate_hz": LaunchConfiguration("external_pose_publish_rate_hz"),
            "input_timeout_s": LaunchConfiguration("external_pose_timeout_s"),
            "adapter_timeout_s": LaunchConfiguration("external_pose_timeout_s"),
            "publish_companion_status": "true",
            "map_frame": LaunchConfiguration("map_frame"),
            "position_offset_m": LaunchConfiguration("position_offset_m"),
            "rpy_offset_rad": LaunchConfiguration("rpy_offset_rad"),
            "optitrack_server": LaunchConfiguration("optitrack_server"),
            "optitrack_port": LaunchConfiguration("optitrack_port"),
            "rigid_body_name": LaunchConfiguration("rigid_body_name"),
            "vrpn_update_freq": LaunchConfiguration("vrpn_update_freq"),
            "vrpn_refresh_freq": LaunchConfiguration("vrpn_refresh_freq"),
            "vrpn_sensor_data_qos": LaunchConfiguration("vrpn_sensor_data_qos"),
            "source_best_effort": LaunchConfiguration("source_best_effort"),
            "use_vrpn_timestamps": LaunchConfiguration("use_vrpn_timestamps"),
            "enable_pose_debug": LaunchConfiguration("enable_pose_debug"),
            "debug_publish_rate_hz": LaunchConfiguration("debug_publish_rate_hz"),
            "debug_history_window_s": LaunchConfiguration("debug_history_window_s"),
            "debug_hold_timeout_s": LaunchConfiguration("debug_hold_timeout_s"),
            "debug_warn_gap_s": LaunchConfiguration("debug_warn_gap_s"),
            "debug_warn_position_error_m": LaunchConfiguration(
                "debug_warn_position_error_m"
            ),
            "debug_warn_yaw_error_deg": LaunchConfiguration("debug_warn_yaw_error_deg"),
        }.items(),
    )

    demo_node = Node(
        package="drone_control_pkg",
        executable="position_hover_demo_sequence_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"), value_type=float
                ),
                "takeoff_altitude_m": ParameterValue(
                    LaunchConfiguration("takeoff_altitude_m"),
                    value_type=float,
                ),
                "takeoff_rate_m_s": ParameterValue(
                    LaunchConfiguration("takeoff_rate_m_s"),
                    value_type=float,
                ),
                "takeoff_strategy": LaunchConfiguration("takeoff_strategy"),
                "hover_duration_s": ParameterValue(
                    LaunchConfiguration("hover_duration_s"),
                    value_type=float,
                ),
                "hover_mode": LaunchConfiguration("hover_mode"),
                "altitude_tolerance_m": ParameterValue(
                    LaunchConfiguration("altitude_tolerance_m"),
                    value_type=float,
                ),
                "start_altitude_limit_m": ParameterValue(
                    LaunchConfiguration("start_altitude_limit_m"),
                    value_type=float,
                ),
                "touchdown_altitude_m": ParameterValue(
                    LaunchConfiguration("touchdown_altitude_m"),
                    value_type=float,
                ),
                "touchdown_dwell_s": ParameterValue(
                    LaunchConfiguration("touchdown_dwell_s"),
                    value_type=float,
                ),
                "stage_timeout_s": ParameterValue(
                    LaunchConfiguration("stage_timeout_s"),
                    value_type=float,
                ),
                "arm_zero_throttle_hold_s": ParameterValue(
                    LaunchConfiguration("arm_zero_throttle_hold_s"),
                    value_type=float,
                ),
                "manual_hover_throttle_center": ParameterValue(
                    LaunchConfiguration("manual_hover_throttle_center"),
                    value_type=float,
                ),
                "local_pose_timeout_s": ParameterValue(
                    LaunchConfiguration("local_pose_timeout_s"),
                    value_type=float,
                ),
                "local_pose_timeout_during_param_sync_s": ParameterValue(
                    LaunchConfiguration("local_pose_timeout_during_param_sync_s"),
                    value_type=float,
                ),
                "state_timeout_s": ParameterValue(
                    LaunchConfiguration("state_timeout_s"),
                    value_type=float,
                ),
                "state_timeout_during_param_sync_s": ParameterValue(
                    LaunchConfiguration("state_timeout_during_param_sync_s"),
                    value_type=float,
                ),
                "connection_loss_timeout_s": ParameterValue(
                    LaunchConfiguration("connection_loss_timeout_s"),
                    value_type=float,
                ),
                "connection_loss_timeout_during_param_sync_s": ParameterValue(
                    LaunchConfiguration(
                        "connection_loss_timeout_during_param_sync_s"
                    ),
                    value_type=float,
                ),
                "companion_status_timeout_s": ParameterValue(
                    LaunchConfiguration("companion_status_timeout_s"),
                    value_type=float,
                ),
                "require_companion_active": ParameterValue(
                    LaunchConfiguration("require_companion_active"),
                    value_type=bool,
                ),
                "max_horizontal_excursion_m": ParameterValue(
                    LaunchConfiguration("max_horizontal_excursion_m"),
                    value_type=float,
                ),
                "restore_takeoff_alt_on_exit": ParameterValue(
                    LaunchConfiguration("restore_takeoff_alt_on_exit"),
                    value_type=bool,
                ),
                "takeoff_param_id": LaunchConfiguration("takeoff_param_id"),
                "param_pull_force": ParameterValue(
                    LaunchConfiguration("param_pull_force"),
                    value_type=bool,
                ),
                "param_pull_retry_delay_s": ParameterValue(
                    LaunchConfiguration("param_pull_retry_delay_s"),
                    value_type=float,
                ),
                "param_sync_timeout_s": ParameterValue(
                    LaunchConfiguration("param_sync_timeout_s"),
                    value_type=float,
                ),
                "use_speed_profile": ParameterValue(
                    LaunchConfiguration("use_speed_profile"),
                    value_type=bool,
                ),
                "speed_profile_config": LaunchConfiguration("speed_profile_config"),
                "restore_speed_profile_on_exit": ParameterValue(
                    LaunchConfiguration("restore_speed_profile_on_exit"),
                    value_type=bool,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [pose_bridge_launch, demo_node]


def generate_launch_description():
    defaults = _load_optitrack_defaults()
    bringup_share = get_package_share_directory("drone_bringup")
    default_speed_profile_config = os.path.join(
        bringup_share,
        "config",
        "indoor_speed_profile.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "drone_id",
                default_value=_default_arg(defaults, "drone_id", ""),
            ),
            DeclareLaunchArgument(
                "mavros_namespace",
                default_value=_default_arg(defaults, "mavros_namespace", "mavros"),
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value=_default_arg(defaults, "pose_source", "optitrack"),
            ),
            DeclareLaunchArgument("source_pose_topic", default_value=""),
            DeclareLaunchArgument(
                "global_origin_latitude_deg",
                default_value=_default_arg(
                    defaults, "global_origin_latitude_deg", 0.0
                ),
            ),
            DeclareLaunchArgument(
                "global_origin_longitude_deg",
                default_value=_default_arg(
                    defaults, "global_origin_longitude_deg", 0.0
                ),
            ),
            DeclareLaunchArgument(
                "global_origin_altitude_m",
                default_value=_default_arg(
                    defaults, "global_origin_altitude_m", 17.1637
                ),
            ),
            DeclareLaunchArgument(
                "home_position_x_m",
                default_value=_default_arg(defaults, "home_position_x_m", 0.0),
            ),
            DeclareLaunchArgument(
                "home_position_y_m",
                default_value=_default_arg(defaults, "home_position_y_m", 0.0),
            ),
            DeclareLaunchArgument(
                "home_position_z_m",
                default_value=_default_arg(defaults, "home_position_z_m", 0.0),
            ),
            DeclareLaunchArgument(
                "home_approach_z_m",
                default_value=_default_arg(defaults, "home_approach_z_m", 1.0),
            ),
            DeclareLaunchArgument(
                "reference_retry_period_s",
                default_value=_default_arg(defaults, "reference_retry_period_s", 1.0),
            ),
            DeclareLaunchArgument("external_pose_publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("external_pose_timeout_s", default_value="0.25"),
            DeclareLaunchArgument(
                "map_frame",
                default_value=_default_arg(defaults, "map_frame", "map"),
            ),
            DeclareLaunchArgument(
                "position_offset_m", default_value="[0.0, 0.0, 0.0]"
            ),
            DeclareLaunchArgument(
                "rpy_offset_rad", default_value="[0.0, 0.0, 0.0]"
            ),
            DeclareLaunchArgument(
                "optitrack_server",
                default_value=_default_arg(
                    defaults, "optitrack_server", "192.168.0.217"
                ),
            ),
            DeclareLaunchArgument(
                "optitrack_port",
                default_value=_default_arg(defaults, "optitrack_port", 3883),
            ),
            DeclareLaunchArgument(
                "rigid_body_name",
                default_value=_default_arg(defaults, "rigid_body_name", ""),
            ),
            DeclareLaunchArgument(
                "vrpn_update_freq",
                default_value=_default_arg(defaults, "vrpn_update_freq", 100.0),
            ),
            DeclareLaunchArgument(
                "vrpn_refresh_freq",
                default_value=_default_arg(defaults, "vrpn_refresh_freq", 1.0),
            ),
            DeclareLaunchArgument(
                "vrpn_sensor_data_qos",
                default_value=_default_arg(defaults, "vrpn_sensor_data_qos", True),
            ),
            DeclareLaunchArgument(
                "source_best_effort",
                default_value=_default_arg(defaults, "source_best_effort", True),
            ),
            DeclareLaunchArgument(
                "use_vrpn_timestamps",
                default_value=_default_arg(defaults, "use_vrpn_timestamps", False),
            ),
            DeclareLaunchArgument("enable_pose_debug", default_value="true"),
            DeclareLaunchArgument("debug_publish_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("debug_history_window_s", default_value="5.0"),
            DeclareLaunchArgument("debug_hold_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("debug_warn_gap_s", default_value="0.25"),
            DeclareLaunchArgument(
                "debug_warn_position_error_m", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "debug_warn_yaw_error_deg", default_value="10.0"
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="0.7"),
            DeclareLaunchArgument("takeoff_rate_m_s", default_value="0.5"),
            DeclareLaunchArgument("takeoff_strategy", default_value="AUTO_MODE"),
            DeclareLaunchArgument("hover_duration_s", default_value="5.0"),
            DeclareLaunchArgument("hover_mode", default_value="HOLD"),
            DeclareLaunchArgument("altitude_tolerance_m", default_value="0.10"),
            DeclareLaunchArgument("start_altitude_limit_m", default_value="0.20"),
            DeclareLaunchArgument("touchdown_altitude_m", default_value="0.15"),
            DeclareLaunchArgument("touchdown_dwell_s", default_value="1.0"),
            DeclareLaunchArgument("stage_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("arm_zero_throttle_hold_s", default_value="1.5"),
            DeclareLaunchArgument("manual_hover_throttle_center", default_value="500.0"),
            DeclareLaunchArgument("local_pose_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "local_pose_timeout_during_param_sync_s", default_value="1.0"
            ),
            DeclareLaunchArgument("state_timeout_s", default_value="2.0"),
            DeclareLaunchArgument(
                "state_timeout_during_param_sync_s", default_value="8.0"
            ),
            DeclareLaunchArgument("connection_loss_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "connection_loss_timeout_during_param_sync_s",
                default_value="3.0",
            ),
            DeclareLaunchArgument("companion_status_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("require_companion_active", default_value="true"),
            DeclareLaunchArgument("max_horizontal_excursion_m", default_value="0.75"),
            DeclareLaunchArgument("restore_takeoff_alt_on_exit", default_value="true"),
            DeclareLaunchArgument("takeoff_param_id", default_value="MIS_TAKEOFF_ALT"),
            DeclareLaunchArgument("param_pull_force", default_value="true"),
            DeclareLaunchArgument("param_pull_retry_delay_s", default_value="1.0"),
            DeclareLaunchArgument("param_sync_timeout_s", default_value="60.0"),
            DeclareLaunchArgument("use_speed_profile", default_value="false"),
            DeclareLaunchArgument(
                "speed_profile_config",
                default_value=default_speed_profile_config,
            ),
            DeclareLaunchArgument(
                "restore_speed_profile_on_exit",
                default_value="true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
