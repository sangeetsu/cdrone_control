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

    tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "tracking_only.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "drone_id": drone_id,
            "source_mode": LaunchConfiguration("tracking_source_mode"),
            "pose_topic": LaunchConfiguration("tracking_pose_topic"),
            "publish_world_track_compare": LaunchConfiguration(
                "publish_world_track_compare"
            ),
            "compare_pose_topic": LaunchConfiguration("compare_pose_topic"),
            "world_track_compare_topic": LaunchConfiguration(
                "world_track_compare_topic"
            ),
            "experiment_tag": LaunchConfiguration("experiment_tag"),
            "model_path": LaunchConfiguration("model_path"),
            "tracker_config_file": LaunchConfiguration("tracker_config_file"),
        }.items(),
    )

    mavros_velocity_node = Node(
        package="drone_control_pkg",
        executable="mavros_velocity_node",
        output="screen",
        parameters=[
            {
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("velocity_rate_hz"), value_type=float
                ),
                "watchdog_timeout_s": ParameterValue(
                    LaunchConfiguration("watchdog_timeout_s"), value_type=float
                ),
                "require_guided_mode": ParameterValue(
                    LaunchConfiguration("require_guided_mode"), value_type=bool
                ),
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    milestone2_demo_node = Node(
        package="drone_control_pkg",
        executable="milestone2_demo_sequence_node",
        output="screen",
        parameters=[
            LaunchConfiguration("demo_config"),
            {
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("follow_rate_hz"), value_type=float
                ),
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "tracks_topic": LaunchConfiguration("tracks_topic"),
                "engagement_state_topic": LaunchConfiguration("engagement_state_topic"),
                "perimeter_config": LaunchConfiguration("perimeter_config"),
                "required_completion_count": ParameterValue(
                    LaunchConfiguration("required_completion_count"), value_type=int
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [
        pose_bridge_launch,
        tracking_launch,
        mavros_velocity_node,
        milestone2_demo_node,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    vision_share = get_package_share_directory("drone_vision_pkg")
    defaults = _load_optitrack_defaults()
    default_tracker_config = os.path.join(
        vision_share,
        "config",
        "tracking_only.yaml",
    )
    default_demo_config = os.path.join(
        bringup_share,
        "config",
        "milestone2_demo.yaml",
    )
    default_perimeter_config = os.path.join(
        bringup_share,
        "config",
        "drone_studio_perimeter.yaml",
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
            DeclareLaunchArgument("tracks_topic", default_value=""),
            DeclareLaunchArgument("engagement_state_topic", default_value=""),
            DeclareLaunchArgument("tracking_source_mode", default_value="direct"),
            DeclareLaunchArgument(
                "tracking_pose_topic",
                default_value=_default_arg(
                    defaults, "ownship_pose_topic", ""
                ),
            ),
            DeclareLaunchArgument(
                "publish_world_track_compare",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "compare_pose_topic",
                default_value=_default_arg(
                    defaults, "compare_pose_topic", "/vrpn_mocap/RigidBody2/pose"
                ),
            ),
            DeclareLaunchArgument("world_track_compare_topic", default_value=""),
            DeclareLaunchArgument("experiment_tag", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=default_tracker_config,
            ),
            DeclareLaunchArgument(
                "demo_config",
                default_value=default_demo_config,
            ),
            DeclareLaunchArgument(
                "required_completion_count",
                default_value="1",
            ),
            DeclareLaunchArgument(
                "perimeter_config",
                default_value=default_perimeter_config,
            ),
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
            DeclareLaunchArgument("velocity_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("watchdog_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("require_guided_mode", default_value="true"),
            DeclareLaunchArgument("follow_rate_hz", default_value="20.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
