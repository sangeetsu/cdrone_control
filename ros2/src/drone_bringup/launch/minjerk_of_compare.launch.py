# flake8: noqa
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


def _cdrone_topic(defaults: dict[str, object], leaf: str) -> str:
    drone_id = str(defaults.get("drone_id", "cdrone")).strip().strip("/")
    namespace = f"/cdrone/{drone_id}" if drone_id else "/cdrone"
    return f"{namespace}/{leaf.strip('/')}"


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
            "drone_namespace": LaunchConfiguration("drone_namespace"),
            "mocap_namespace": LaunchConfiguration("mocap_namespace"),
            "pose_source": LaunchConfiguration("pose_source"),
            "source_pose_topic": LaunchConfiguration("source_pose_topic"),
            "enable_reference_setup": "true",
            "map_frame": LaunchConfiguration("comparison_frame_id"),
            "frame_rpy_rad": LaunchConfiguration("frame_rpy_rad"),
            "position_offset_m": LaunchConfiguration("position_offset_m"),
            "rpy_offset_rad": LaunchConfiguration("rpy_offset_rad"),
            "optitrack_server": LaunchConfiguration("optitrack_server"),
            "optitrack_port": LaunchConfiguration("optitrack_port"),
            "rigid_body_name": LaunchConfiguration("rigid_body_name"),
            "source_best_effort": LaunchConfiguration("source_best_effort"),
            "use_vrpn_timestamps": LaunchConfiguration("use_vrpn_timestamps"),
            "enable_pose_debug": LaunchConfiguration("enable_pose_debug"),
        }.items(),
    )

    mission_node = Node(
        package="drone_control_pkg",
        executable="minjerk_waypoint_mission_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "demo_mode": "waypoints",
                "waypoints_config": LaunchConfiguration("waypoints_config"),
                "mocap_pose_topic": LaunchConfiguration("mocap_pose_topic"),
                "source_best_effort": ParameterValue(
                    LaunchConfiguration("source_best_effort"),
                    value_type=bool,
                ),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("publish_rate_hz"),
                    value_type=float,
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
                "use_speed_profile": ParameterValue(
                    LaunchConfiguration("use_speed_profile"),
                    value_type=bool,
                ),
                "speed_profile_config": LaunchConfiguration("speed_profile_config"),
                "restore_speed_profile_on_exit": ParameterValue(
                    LaunchConfiguration("restore_speed_profile_on_exit"),
                    value_type=bool,
                ),
                "offboard_hold_test": ParameterValue(
                    LaunchConfiguration("offboard_hold_test"),
                    value_type=bool,
                ),
                "goal_hold_duration_s": ParameterValue(
                    LaunchConfiguration("goal_hold_duration_s"),
                    value_type=float,
                ),
                "goal_position_tolerance_m": ParameterValue(
                    LaunchConfiguration("goal_position_tolerance_m"),
                    value_type=float,
                ),
                "offboard_setpoint_warmup_s": ParameterValue(
                    LaunchConfiguration("offboard_setpoint_warmup_s"),
                    value_type=float,
                ),
                "enable_perimeter_guard": ParameterValue(
                    LaunchConfiguration("enable_perimeter_guard"),
                    value_type=bool,
                ),
                "perimeter_config": LaunchConfiguration("perimeter_config"),
                "status_topic": LaunchConfiguration("mission_state_topic"),
                "start_service": LaunchConfiguration("start_service"),
                "abort_service": LaunchConfiguration("abort_service"),
                "position_setpoint_topic": LaunchConfiguration("setpoint_topic"),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    logger_node = Node(
        package="drone_control_pkg",
        executable="mocap_of_compare_logger_node",
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "mocap_pose_topic": LaunchConfiguration("mocap_pose_topic"),
                "flow_rad_topic": LaunchConfiguration("flow_rad_topic"),
                "flow_range_topic": LaunchConfiguration("flow_range_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "setpoint_topic": LaunchConfiguration("setpoint_topic"),
                "mission_state_topic": LaunchConfiguration("mission_state_topic"),
                "fused_pose_topic": LaunchConfiguration("fused_pose_topic"),
                "imu_only_pose_topic": LaunchConfiguration("imu_only_pose_topic"),
                "output_dir": LaunchConfiguration("output_dir"),
                "run_id": LaunchConfiguration("run_id"),
                "comparison_frame_id": LaunchConfiguration("comparison_frame_id"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("logger_publish_rate_hz"),
                    value_type=float,
                ),
                "source_best_effort": ParameterValue(
                    LaunchConfiguration("source_best_effort"),
                    value_type=bool,
                ),
                "quality_min": ParameterValue(
                    LaunchConfiguration("quality_min"),
                    value_type=int,
                ),
                "range_min_m": ParameterValue(
                    LaunchConfiguration("range_min_m"),
                    value_type=float,
                ),
                "range_max_m": ParameterValue(
                    LaunchConfiguration("range_max_m"),
                    value_type=float,
                ),
                "gyro_compensation_gain": ParameterValue(
                    LaunchConfiguration("gyro_compensation_gain"),
                    value_type=float,
                ),
                "flow_scale_x": ParameterValue(
                    LaunchConfiguration("flow_scale_x"),
                    value_type=float,
                ),
                "flow_scale_y": ParameterValue(
                    LaunchConfiguration("flow_scale_y"),
                    value_type=float,
                ),
                "fusion_enabled": ParameterValue(
                    LaunchConfiguration("fusion_enabled"),
                    value_type=bool,
                ),
                "imu_only_enabled": ParameterValue(
                    LaunchConfiguration("imu_only_enabled"),
                    value_type=bool,
                ),
                "of_yaw_source": LaunchConfiguration("of_yaw_source"),
                "fusion_flow_position_weight": ParameterValue(
                    LaunchConfiguration("fusion_flow_position_weight"),
                    value_type=float,
                ),
                "fusion_flow_velocity_weight": ParameterValue(
                    LaunchConfiguration("fusion_flow_velocity_weight"),
                    value_type=float,
                ),
                "fusion_range_z_weight": ParameterValue(
                    LaunchConfiguration("fusion_range_z_weight"),
                    value_type=float,
                ),
                "fusion_subtract_gravity": ParameterValue(
                    LaunchConfiguration("fusion_subtract_gravity"),
                    value_type=bool,
                ),
                "fusion_gravity_mps2": ParameterValue(
                    LaunchConfiguration("fusion_gravity_mps2"),
                    value_type=float,
                ),
                "fusion_accel_deadband_mps2": ParameterValue(
                    LaunchConfiguration("fusion_accel_deadband_mps2"),
                    value_type=float,
                ),
                "fusion_max_accel_mps2": ParameterValue(
                    LaunchConfiguration("fusion_max_accel_mps2"),
                    value_type=float,
                ),
                "fusion_max_velocity_mps": ParameterValue(
                    LaunchConfiguration("fusion_max_velocity_mps"),
                    value_type=float,
                ),
                "fusion_max_imu_dt_s": ParameterValue(
                    LaunchConfiguration("fusion_max_imu_dt_s"),
                    value_type=float,
                ),
                "fusion_velocity_decay_per_s": ParameterValue(
                    LaunchConfiguration("fusion_velocity_decay_per_s"),
                    value_type=float,
                ),
                "fusion_accel_noise_mps2": ParameterValue(
                    LaunchConfiguration("fusion_accel_noise_mps2"),
                    value_type=float,
                ),
                "fusion_accel_bias_rw_mps3": ParameterValue(
                    LaunchConfiguration("fusion_accel_bias_rw_mps3"),
                    value_type=float,
                ),
                "fusion_flow_velocity_noise_mps": ParameterValue(
                    LaunchConfiguration("fusion_flow_velocity_noise_mps"),
                    value_type=float,
                ),
                "fusion_range_noise_m": ParameterValue(
                    LaunchConfiguration("fusion_range_noise_m"),
                    value_type=float,
                ),
                "fusion_accel_lpf_tau_s": ParameterValue(
                    LaunchConfiguration("fusion_accel_lpf_tau_s"),
                    value_type=float,
                ),
                "fusion_innovation_gate_nis": ParameterValue(
                    LaunchConfiguration("fusion_innovation_gate_nis"),
                    value_type=float,
                ),
                "fusion_range_innovation_gate_nis": ParameterValue(
                    LaunchConfiguration("fusion_range_innovation_gate_nis"),
                    value_type=float,
                ),
                "fusion_max_sensor_age_s": ParameterValue(
                    LaunchConfiguration("fusion_max_sensor_age_s"),
                    value_type=float,
                ),
                "fusion_reorder_tolerance_s": ParameterValue(
                    LaunchConfiguration("fusion_reorder_tolerance_s"),
                    value_type=float,
                ),
                "fusion_max_tilt_rad": ParameterValue(
                    LaunchConfiguration("fusion_max_tilt_rad"),
                    value_type=float,
                ),
                "fusion_max_flow_gap_s": ParameterValue(
                    LaunchConfiguration("fusion_max_flow_gap_s"),
                    value_type=float,
                ),
                "fusion_flow_gap_noise_scale": ParameterValue(
                    LaunchConfiguration("fusion_flow_gap_noise_scale"),
                    value_type=float,
                ),
                "fusion_flow_quality_noise_scale": ParameterValue(
                    LaunchConfiguration("fusion_flow_quality_noise_scale"),
                    value_type=float,
                ),
                "fusion_flow_range_noise_scale": ParameterValue(
                    LaunchConfiguration("fusion_flow_range_noise_scale"),
                    value_type=float,
                ),
                "fusion_gyro_fallback_noise_scale": ParameterValue(
                    LaunchConfiguration("fusion_gyro_fallback_noise_scale"),
                    value_type=float,
                ),
                "fusion_gyro_coverage_tolerance_s": ParameterValue(
                    LaunchConfiguration("fusion_gyro_coverage_tolerance_s"),
                    value_type=float,
                ),
                "fusion_gyro_buffer_duration_s": ParameterValue(
                    LaunchConfiguration("fusion_gyro_buffer_duration_s"),
                    value_type=float,
                ),
                "fusion_flow_sensor_yaw_rad": ParameterValue(
                    LaunchConfiguration("fusion_flow_sensor_yaw_rad"),
                    value_type=float,
                ),
                "fusion_initial_position_std_m": ParameterValue(
                    LaunchConfiguration("fusion_initial_position_std_m"),
                    value_type=float,
                ),
                "fusion_initial_velocity_std_mps": ParameterValue(
                    LaunchConfiguration("fusion_initial_velocity_std_mps"),
                    value_type=float,
                ),
                "fusion_initial_accel_bias_std_mps2": ParameterValue(
                    LaunchConfiguration("fusion_initial_accel_bias_std_mps2"),
                    value_type=float,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [pose_bridge_launch, mission_node, logger_node]


def generate_launch_description():
    defaults = load_drone_launch_defaults()
    bringup_share = get_package_share_directory("drone_bringup")
    default_waypoints_config = os.path.join(
        bringup_share,
        "config",
        "minjerk_waypoints.yaml",
    )
    default_perimeter_config = os.path.join(
        bringup_share,
        "config",
        "drone_studio_perimeter.yaml",
    )
    default_state_topic = _cdrone_topic(
        defaults,
        "demo/minjerk_waypoints_state",
    )
    default_start_service = _cdrone_topic(
        defaults,
        "demo/minjerk_waypoints_start",
    )
    default_abort_service = _cdrone_topic(
        defaults,
        "demo/minjerk_waypoints_abort",
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
                "drone_namespace",
                default_value=_default_arg(defaults, "drone_namespace", "/cdrone"),
            ),
            DeclareLaunchArgument(
                "mocap_namespace",
                default_value=_default_arg(
                    defaults,
                    "mocap_namespace",
                    "/cdrone/vrpn_mocap",
                ),
            ),
            DeclareLaunchArgument(
                "pose_source",
                default_value=_default_arg(defaults, "pose_source", "optitrack"),
            ),
            DeclareLaunchArgument("source_pose_topic", default_value=""),
            DeclareLaunchArgument(
                "mocap_pose_topic",
                default_value=_default_arg(defaults, "ownship_pose_topic", ""),
            ),
            DeclareLaunchArgument(
                "comparison_frame_id",
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
            DeclareLaunchArgument("position_offset_m", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("rpy_offset_rad", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument(
                "optitrack_server",
                default_value=_default_arg(
                    defaults,
                    "optitrack_server",
                    "192.168.0.217",
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
            DeclareLaunchArgument("source_best_effort", default_value="true"),
            DeclareLaunchArgument("use_vrpn_timestamps", default_value="false"),
            DeclareLaunchArgument("enable_pose_debug", default_value="true"),
            DeclareLaunchArgument(
                "waypoints_config",
                default_value=default_waypoints_config,
            ),
            DeclareLaunchArgument(
                "perimeter_config",
                default_value=default_perimeter_config,
            ),
            DeclareLaunchArgument("enable_perimeter_guard", default_value="true"),
            DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="1.2"),
            DeclareLaunchArgument("takeoff_rate_m_s", default_value="0.5"),
            DeclareLaunchArgument("takeoff_strategy", default_value="AUTO_MODE"),
            DeclareLaunchArgument("use_speed_profile", default_value="false"),
            DeclareLaunchArgument("speed_profile_config", default_value=""),
            DeclareLaunchArgument("restore_speed_profile_on_exit", default_value="true"),
            DeclareLaunchArgument("offboard_hold_test", default_value="false"),
            DeclareLaunchArgument("goal_hold_duration_s", default_value="2.0"),
            DeclareLaunchArgument("goal_position_tolerance_m", default_value="0.15"),
            DeclareLaunchArgument("offboard_setpoint_warmup_s", default_value="1.5"),
            DeclareLaunchArgument(
                "setpoint_topic",
                default_value=_default_arg(
                    defaults,
                    "setpoint_topic",
                    f"{defaults.get('mavros_namespace', '/mavros')}"
                    "/setpoint_position/local",
                ),
            ),
            DeclareLaunchArgument(
                "mission_state_topic",
                default_value=default_state_topic,
            ),
            DeclareLaunchArgument("start_service", default_value=default_start_service),
            DeclareLaunchArgument("abort_service", default_value=default_abort_service),
            DeclareLaunchArgument(
                "flow_rad_topic",
                default_value=_default_arg(
                    defaults,
                    "flow_rad_topic",
                    f"{defaults.get('mavros_namespace', '/mavros')}"
                    "/px4flow/raw/optical_flow_rad",
                ),
            ),
            DeclareLaunchArgument(
                "flow_range_topic",
                default_value=_default_arg(
                    defaults,
                    "flow_range_topic",
                    f"{defaults.get('mavros_namespace', '/mavros')}"
                    "/px4flow/ground_distance",
                ),
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value=_default_arg(
                    defaults,
                    "imu_topic",
                    f"{defaults.get('mavros_namespace', '/mavros')}"
                    "/imu/data",
                ),
            ),
            DeclareLaunchArgument(
                "fused_pose_topic",
                default_value=_cdrone_topic(defaults, "of_compare/fused_pose"),
            ),
            DeclareLaunchArgument(
                "imu_only_pose_topic",
                default_value=_cdrone_topic(defaults, "of_compare/imu_only_pose"),
            ),
            DeclareLaunchArgument("output_dir", default_value="flight_logs/of_compare"),
            DeclareLaunchArgument("run_id", default_value=""),
            DeclareLaunchArgument("logger_publish_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("quality_min", default_value="10"),
            DeclareLaunchArgument("range_min_m", default_value="0.2"),
            DeclareLaunchArgument("range_max_m", default_value="5.0"),
            DeclareLaunchArgument("gyro_compensation_gain", default_value="1.0"),
            DeclareLaunchArgument("flow_scale_x", default_value="1.0"),
            DeclareLaunchArgument("flow_scale_y", default_value="1.0"),
            DeclareLaunchArgument("fusion_enabled", default_value="true"),
            DeclareLaunchArgument("imu_only_enabled", default_value="true"),
            DeclareLaunchArgument("of_yaw_source", default_value="imu"),
            DeclareLaunchArgument(
                "fusion_flow_position_weight",
                default_value="0.75",
            ),
            DeclareLaunchArgument(
                "fusion_flow_velocity_weight",
                default_value="0.50",
            ),
            DeclareLaunchArgument("fusion_range_z_weight", default_value="0.80"),
            DeclareLaunchArgument("fusion_subtract_gravity", default_value="true"),
            DeclareLaunchArgument("fusion_gravity_mps2", default_value="9.80665"),
            DeclareLaunchArgument(
                "fusion_accel_deadband_mps2",
                default_value="0.05",
            ),
            DeclareLaunchArgument("fusion_max_accel_mps2", default_value="6.0"),
            DeclareLaunchArgument("fusion_max_velocity_mps", default_value="4.0"),
            DeclareLaunchArgument("fusion_max_imu_dt_s", default_value="0.10"),
            DeclareLaunchArgument(
                "fusion_velocity_decay_per_s",
                default_value="0.04",
            ),
            DeclareLaunchArgument("fusion_accel_noise_mps2", default_value="0.80"),
            DeclareLaunchArgument(
                "fusion_accel_bias_rw_mps3",
                default_value="0.03",
            ),
            DeclareLaunchArgument(
                "fusion_flow_velocity_noise_mps",
                default_value="0.20",
            ),
            DeclareLaunchArgument("fusion_range_noise_m", default_value="0.08"),
            DeclareLaunchArgument("fusion_accel_lpf_tau_s", default_value="0.08"),
            DeclareLaunchArgument(
                "fusion_innovation_gate_nis",
                default_value="9.21",
            ),
            DeclareLaunchArgument(
                "fusion_range_innovation_gate_nis",
                default_value="6.635",
            ),
            DeclareLaunchArgument(
                "fusion_max_sensor_age_s",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "fusion_reorder_tolerance_s",
                default_value="0.02",
            ),
            DeclareLaunchArgument(
                "fusion_max_tilt_rad",
                default_value="0.7853981633974483",
            ),
            DeclareLaunchArgument("fusion_max_flow_gap_s", default_value="0.50"),
            DeclareLaunchArgument(
                "fusion_flow_gap_noise_scale",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "fusion_flow_quality_noise_scale",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "fusion_flow_range_noise_scale",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "fusion_gyro_fallback_noise_scale",
                default_value="2.0",
            ),
            DeclareLaunchArgument(
                "fusion_gyro_coverage_tolerance_s",
                default_value="0.005",
            ),
            DeclareLaunchArgument(
                "fusion_gyro_buffer_duration_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "fusion_flow_sensor_yaw_rad",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "fusion_initial_position_std_m",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "fusion_initial_velocity_std_mps",
                default_value="0.50",
            ),
            DeclareLaunchArgument(
                "fusion_initial_accel_bias_std_mps2",
                default_value="0.30",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
