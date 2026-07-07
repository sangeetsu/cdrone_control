import os

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import (
    default_arg as _default_arg,
    load_drone_launch_defaults,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_optitrack_defaults() -> dict[str, object]:
    return load_drone_launch_defaults()


def _join_topic(namespace: str, leaf: str) -> str:
    namespace = str(namespace or "").strip()
    if namespace and not namespace.startswith("/"):
        namespace = "/" + namespace
    namespace = namespace.rstrip("/")
    leaf = "/" + str(leaf or "").strip().lstrip("/")
    return f"{namespace}{leaf}" if namespace else leaf


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    drone_namespace = LaunchConfiguration("drone_namespace")
    mocap_namespace = LaunchConfiguration("mocap_namespace").perform(context).strip()
    drone_id = LaunchConfiguration("drone_id")
    pose_source = LaunchConfiguration("pose_source").perform(context).strip().lower()
    source_pose_topic = LaunchConfiguration("source_pose_topic").perform(context).strip()
    rigid_body_name = LaunchConfiguration("rigid_body_name").perform(context).strip()
    default_optitrack_topic = _join_topic(mocap_namespace, f"{rigid_body_name}/pose")

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    nodes = [drone_launch]
    nodes.append(
        Node(
            package="drone_control_pkg",
            executable="drone_setup_node",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_reference_setup")),
            parameters=[
                {
                    "mavros_namespace": mavros_namespace,
                    "global_origin_latitude_deg": ParameterValue(
                        LaunchConfiguration("global_origin_latitude_deg"),
                        value_type=float,
                    ),
                    "global_origin_longitude_deg": ParameterValue(
                        LaunchConfiguration("global_origin_longitude_deg"),
                        value_type=float,
                    ),
                    "global_origin_altitude_m": ParameterValue(
                        LaunchConfiguration("global_origin_altitude_m"),
                        value_type=float,
                    ),
                    "home_position_x_m": ParameterValue(
                        LaunchConfiguration("home_position_x_m"), value_type=float
                    ),
                    "home_position_y_m": ParameterValue(
                        LaunchConfiguration("home_position_y_m"), value_type=float
                    ),
                    "home_position_z_m": ParameterValue(
                        LaunchConfiguration("home_position_z_m"), value_type=float
                    ),
                    "home_approach_z_m": ParameterValue(
                        LaunchConfiguration("home_approach_z_m"), value_type=float
                    ),
                    "retry_period_s": ParameterValue(
                        LaunchConfiguration("reference_retry_period_s"),
                        value_type=float,
                    ),
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )

    if pose_source == "optitrack":
        vrpn_share = get_package_share_directory("vrpn_mocap")
        nodes.append(
            Node(
                package="vrpn_mocap",
                executable="client_node",
                namespace=LaunchConfiguration("mocap_namespace"),
                name="vrpn_mocap_client_node",
                output="screen",
                parameters=[
                    os.path.join(vrpn_share, "config", "client.yaml"),
                    {
                        "server": LaunchConfiguration("optitrack_server"),
                        "port": ParameterValue(
                            LaunchConfiguration("optitrack_port"), value_type=int
                        ),
                        "frame_id": LaunchConfiguration("map_frame"),
                        "update_freq": ParameterValue(
                            LaunchConfiguration("vrpn_update_freq"), value_type=float
                        ),
                        "refresh_freq": ParameterValue(
                            LaunchConfiguration("vrpn_refresh_freq"), value_type=float
                        ),
                        "sensor_data_qos": ParameterValue(
                            LaunchConfiguration("vrpn_sensor_data_qos"),
                            value_type=bool,
                        ),
                        "use_vrpn_timestamps": ParameterValue(
                            LaunchConfiguration("use_vrpn_timestamps"),
                            value_type=bool,
                        ),
                    }
                ],
                arguments=["--ros-args", "--log-level", log_level],
            )
        )
        adapter_source_topic = source_pose_topic or default_optitrack_topic
    elif pose_source == "realsense_pose":
        adapter_source_topic = source_pose_topic or "/visual_slam/tracking/vo_pose"
    elif pose_source == "topic":
        adapter_source_topic = source_pose_topic or "/external_pose/source_pose"
    else:
        raise ValueError(
            "pose_source must be one of: optitrack, realsense_pose, topic"
        )

    nodes.append(
        Node(
            package="drone_control_pkg",
            executable="external_pose_adapter_node",
            namespace=drone_namespace,
            output="screen",
            parameters=[
                {
                    "drone_id": drone_id,
                    "source_pose_topic": adapter_source_topic,
                    "output_pose_topic": LaunchConfiguration("external_pose_topic"),
                    "map_frame": LaunchConfiguration("map_frame"),
                    "frame_rpy_rad": LaunchConfiguration("frame_rpy_rad"),
                    "position_offset_m": LaunchConfiguration("position_offset_m"),
                    "rpy_offset_rad": LaunchConfiguration("rpy_offset_rad"),
                    "source_best_effort": ParameterValue(
                        LaunchConfiguration("source_best_effort"), value_type=bool
                    ),
                    "timeout_s": ParameterValue(
                        LaunchConfiguration("adapter_timeout_s"), value_type=float
                    ),
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )
    nodes.append(
        Node(
            package="drone_control_pkg",
            executable="external_pose_bridge_node",
            namespace=drone_namespace,
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
                        LaunchConfiguration("publish_companion_status"),
                        value_type=bool,
                    ),
                    "input_pose_topic": LaunchConfiguration("bridge_input_pose_topic"),
                    "legacy_input_pose_topic": LaunchConfiguration(
                        "legacy_input_pose_topic"
                    ),
                    "output_pose_topic": LaunchConfiguration("output_pose_topic"),
                    "restamp_with_local_clock": ParameterValue(
                        LaunchConfiguration("restamp_with_local_clock"),
                        value_type=bool,
                    ),
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )
    nodes.append(
        Node(
            package="drone_control_pkg",
            executable="external_pose_debug_node",
            namespace=drone_namespace,
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_pose_debug")),
            parameters=[
                {
                    "drone_id": drone_id,
                    "mavros_namespace": mavros_namespace,
                    "source_pose_topic": adapter_source_topic,
                    "adapter_output_pose_topic": LaunchConfiguration("external_pose_topic"),
                    "vision_pose_topic": LaunchConfiguration("output_pose_topic"),
                    "publish_rate_hz": ParameterValue(
                        LaunchConfiguration("debug_publish_rate_hz"), value_type=float
                    ),
                    "history_window_s": ParameterValue(
                        LaunchConfiguration("debug_history_window_s"), value_type=float
                    ),
                    "hold_timeout_s": ParameterValue(
                        LaunchConfiguration("debug_hold_timeout_s"), value_type=float
                    ),
                    "warn_gap_s": ParameterValue(
                        LaunchConfiguration("debug_warn_gap_s"), value_type=float
                    ),
                    "warn_position_error_m": ParameterValue(
                        LaunchConfiguration("debug_warn_position_error_m"),
                        value_type=float,
                    ),
                    "warn_yaw_error_deg": ParameterValue(
                        LaunchConfiguration("debug_warn_yaw_error_deg"),
                        value_type=float,
                    ),
                    "source_best_effort": ParameterValue(
                        LaunchConfiguration("source_best_effort"), value_type=bool
                    ),
                }
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
    )

    return nodes


def generate_launch_description():
    defaults = _load_optitrack_defaults()

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
                "drone_namespace",
                default_value=_default_arg(defaults, "drone_namespace", "/cdrone"),
            ),
            DeclareLaunchArgument(
                "mocap_namespace",
                default_value=_default_arg(
                    defaults, "mocap_namespace", "/cdrone/vrpn_mocap"
                ),
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value=_default_arg(defaults, "pose_source", "optitrack"),
            ),
            DeclareLaunchArgument("source_pose_topic", default_value=""),
            DeclareLaunchArgument("external_pose_topic", default_value=""),
            DeclareLaunchArgument("bridge_input_pose_topic", default_value=""),
            DeclareLaunchArgument("legacy_input_pose_topic", default_value=""),
            DeclareLaunchArgument("output_pose_topic", default_value=""),
            DeclareLaunchArgument("enable_reference_setup", default_value="true"),
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
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("input_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("adapter_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("publish_companion_status", default_value="true"),
            DeclareLaunchArgument("restamp_with_local_clock", default_value="false"),
            DeclareLaunchArgument(
                "map_frame",
                default_value=_default_arg(defaults, "map_frame", "map"),
            ),
            DeclareLaunchArgument(
                "frame_rpy_rad",
                default_value=_default_arg(
                    defaults,
                    "frame_rpy_rad",
                    "[0.0, 0.0, 0.0]",
                ),
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
            OpaqueFunction(function=launch_setup),
        ]
    )
