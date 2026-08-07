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
    drone_namespace = LaunchConfiguration("drone_namespace")
    external_pose_topic = LaunchConfiguration("external_pose_topic")

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    reference_setup_node = Node(
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
                    LaunchConfiguration("home_position_x_m"),
                    value_type=float,
                ),
                "home_position_y_m": ParameterValue(
                    LaunchConfiguration("home_position_y_m"),
                    value_type=float,
                ),
                "home_position_z_m": ParameterValue(
                    LaunchConfiguration("home_position_z_m"),
                    value_type=float,
                ),
                "home_approach_z_m": ParameterValue(
                    LaunchConfiguration("home_approach_z_m"),
                    value_type=float,
                ),
                "retry_period_s": ParameterValue(
                    LaunchConfiguration("reference_retry_period_s"),
                    value_type=float,
                ),
                "assume_global_origin_after_publish": ParameterValue(
                    LaunchConfiguration("assume_global_origin_after_publish"),
                    value_type=bool,
                ),
                "assume_home_position_after_publish": ParameterValue(
                    LaunchConfiguration("assume_home_position_after_publish"),
                    value_type=bool,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    estimator_node = Node(
        package="drone_control_pkg",
        executable="optical_flow_hover_estimator_node",
        namespace=drone_namespace,
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("external_pose_publish_rate_hz"),
                    value_type=float,
                ),
                "map_frame": LaunchConfiguration("map_frame"),
                "flow_rad_topic": LaunchConfiguration("flow_rad_topic"),
                "flow_range_topic": LaunchConfiguration("flow_range_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "external_pose_topic": external_pose_topic,
                "debug_pose_topic": LaunchConfiguration("debug_pose_topic"),
                "status_topic": LaunchConfiguration("estimator_status_topic"),
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
                "max_imu_age_s": ParameterValue(
                    LaunchConfiguration("max_imu_age_s"),
                    value_type=float,
                ),
                "max_range_age_s": ParameterValue(
                    LaunchConfiguration("max_range_age_s"),
                    value_type=float,
                ),
                "max_flow_age_s": ParameterValue(
                    LaunchConfiguration("max_flow_age_s"),
                    value_type=float,
                ),
                "allow_near_ground_flow": ParameterValue(
                    LaunchConfiguration("allow_near_ground_flow"),
                    value_type=bool,
                ),
                "near_ground_range_m": ParameterValue(
                    LaunchConfiguration("near_ground_range_m"),
                    value_type=float,
                ),
                "allow_missing_flow_gyro": ParameterValue(
                    LaunchConfiguration("allow_missing_flow_gyro"),
                    value_type=bool,
                ),
                "allow_imu_only_warmup": ParameterValue(
                    LaunchConfiguration("allow_imu_only_warmup"),
                    value_type=bool,
                ),
                "imu_only_warmup_s": ParameterValue(
                    LaunchConfiguration("imu_only_warmup_s"),
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

    bridge_node = Node(
        package="drone_control_pkg",
        executable="external_pose_bridge_node",
        namespace=drone_namespace,
        output="screen",
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("external_pose_publish_rate_hz"),
                    value_type=float,
                ),
                "input_timeout_s": ParameterValue(
                    LaunchConfiguration("external_pose_timeout_s"),
                    value_type=float,
                ),
                "publish_companion_status": True,
                "input_pose_topic": external_pose_topic,
                "output_pose_topic": LaunchConfiguration("vision_pose_topic"),
                "restamp_with_local_clock": ParameterValue(
                    LaunchConfiguration("restamp_with_local_clock"),
                    value_type=bool,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    debug_node = Node(
        package="drone_control_pkg",
        executable="external_pose_debug_node",
        namespace=drone_namespace,
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_pose_debug")),
        parameters=[
            {
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "source_pose_topic": LaunchConfiguration("debug_pose_topic"),
                "adapter_output_pose_topic": external_pose_topic,
                "vision_pose_topic": LaunchConfiguration("vision_pose_topic"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("debug_publish_rate_hz"),
                    value_type=float,
                ),
                "history_window_s": ParameterValue(
                    LaunchConfiguration("debug_history_window_s"),
                    value_type=float,
                ),
                "hold_timeout_s": ParameterValue(
                    LaunchConfiguration("debug_hold_timeout_s"),
                    value_type=float,
                ),
                "warn_gap_s": ParameterValue(
                    LaunchConfiguration("debug_warn_gap_s"),
                    value_type=float,
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
                    LaunchConfiguration("source_best_effort"),
                    value_type=bool,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
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
                    LaunchConfiguration("connection_loss_timeout_during_param_sync_s"),
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
                "local_pose_topic": LaunchConfiguration("demo_local_pose_topic"),
                "require_global_origin": ParameterValue(
                    LaunchConfiguration("require_global_origin"),
                    value_type=bool,
                ),
                "allow_synthetic_home_position": ParameterValue(
                    LaunchConfiguration("allow_synthetic_home_position"),
                    value_type=bool,
                ),
                "synthetic_home_position_z_m": ParameterValue(
                    LaunchConfiguration("synthetic_home_position_z_m"),
                    value_type=float,
                ),
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return [
        drone_launch,
        reference_setup_node,
        estimator_node,
        bridge_node,
        debug_node,
        demo_node,
    ]


def generate_launch_description():
    defaults = load_drone_launch_defaults()
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
                "drone_namespace",
                default_value=_default_arg(defaults, "drone_namespace", "/cdrone"),
            ),
            DeclareLaunchArgument(
                "external_pose_topic",
                default_value=_cdrone_topic(
                    defaults,
                    "external_pose/input_pose",
                ),
            ),
            DeclareLaunchArgument(
                "debug_pose_topic",
                default_value=_cdrone_topic(defaults, "of_hover/fused_pose"),
            ),
            DeclareLaunchArgument(
                "estimator_status_topic",
                default_value=_cdrone_topic(defaults, "of_hover/status"),
            ),
            DeclareLaunchArgument("vision_pose_topic", default_value=""),
            DeclareLaunchArgument(
                "demo_local_pose_topic",
                default_value=_cdrone_topic(defaults, "of_hover/fused_pose"),
            ),
            DeclareLaunchArgument("flow_rad_topic", default_value=""),
            DeclareLaunchArgument("flow_range_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value=""),
            DeclareLaunchArgument("external_pose_publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("external_pose_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("restamp_with_local_clock", default_value="false"),
            DeclareLaunchArgument("enable_reference_setup", default_value="true"),
            DeclareLaunchArgument(
                "global_origin_latitude_deg",
                default_value=_default_arg(defaults, "global_origin_latitude_deg", 0.0),
            ),
            DeclareLaunchArgument(
                "global_origin_longitude_deg",
                default_value=_default_arg(defaults, "global_origin_longitude_deg", 0.0),
            ),
            DeclareLaunchArgument(
                "global_origin_altitude_m",
                default_value=_default_arg(defaults, "global_origin_altitude_m", 17.1637),
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
            DeclareLaunchArgument(
                "map_frame",
                default_value=_default_arg(defaults, "map_frame", "map"),
            ),
            DeclareLaunchArgument(
                "source_best_effort",
                default_value=_default_arg(defaults, "source_best_effort", True),
            ),
            DeclareLaunchArgument("quality_min", default_value="10"),
            DeclareLaunchArgument("range_min_m", default_value="0.05"),
            DeclareLaunchArgument("range_max_m", default_value="5.0"),
            DeclareLaunchArgument("max_imu_age_s", default_value="0.25"),
            DeclareLaunchArgument("max_range_age_s", default_value="0.25"),
            DeclareLaunchArgument("max_flow_age_s", default_value="0.50"),
            DeclareLaunchArgument("allow_near_ground_flow", default_value="true"),
            DeclareLaunchArgument("near_ground_range_m", default_value="0.05"),
            DeclareLaunchArgument("allow_missing_flow_gyro", default_value="true"),
            DeclareLaunchArgument("allow_imu_only_warmup", default_value="true"),
            DeclareLaunchArgument("imu_only_warmup_s", default_value="1.0"),
            DeclareLaunchArgument("gyro_compensation_gain", default_value="1.0"),
            DeclareLaunchArgument("flow_scale_x", default_value="1.0"),
            DeclareLaunchArgument("flow_scale_y", default_value="1.0"),
            DeclareLaunchArgument("fusion_flow_position_weight", default_value="0.75"),
            DeclareLaunchArgument("fusion_flow_velocity_weight", default_value="0.50"),
            DeclareLaunchArgument("fusion_range_z_weight", default_value="0.80"),
            DeclareLaunchArgument("fusion_subtract_gravity", default_value="true"),
            DeclareLaunchArgument("fusion_gravity_mps2", default_value="9.80665"),
            DeclareLaunchArgument("fusion_accel_deadband_mps2", default_value="0.05"),
            DeclareLaunchArgument("fusion_max_accel_mps2", default_value="6.0"),
            DeclareLaunchArgument("fusion_max_velocity_mps", default_value="4.0"),
            DeclareLaunchArgument("fusion_max_imu_dt_s", default_value="0.10"),
            DeclareLaunchArgument("fusion_velocity_decay_per_s", default_value="0.04"),
            DeclareLaunchArgument("fusion_accel_noise_mps2", default_value="0.80"),
            DeclareLaunchArgument("fusion_accel_bias_rw_mps3", default_value="0.03"),
            DeclareLaunchArgument("fusion_flow_velocity_noise_mps", default_value="0.20"),
            DeclareLaunchArgument("fusion_range_noise_m", default_value="0.08"),
            DeclareLaunchArgument("fusion_accel_lpf_tau_s", default_value="0.08"),
            DeclareLaunchArgument("fusion_innovation_gate_nis", default_value="9.21"),
            DeclareLaunchArgument(
                "fusion_range_innovation_gate_nis",
                default_value="6.635",
            ),
            DeclareLaunchArgument("fusion_max_sensor_age_s", default_value="0.25"),
            DeclareLaunchArgument("fusion_reorder_tolerance_s", default_value="0.02"),
            DeclareLaunchArgument(
                "fusion_max_tilt_rad",
                default_value="0.7853981633974483",
            ),
            DeclareLaunchArgument("fusion_max_flow_gap_s", default_value="0.50"),
            DeclareLaunchArgument("fusion_flow_gap_noise_scale", default_value="0.25"),
            DeclareLaunchArgument(
                "fusion_flow_quality_noise_scale",
                default_value="3.0",
            ),
            DeclareLaunchArgument("fusion_flow_range_noise_scale", default_value="1.0"),
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
            DeclareLaunchArgument("fusion_flow_sensor_yaw_rad", default_value="0.0"),
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
            DeclareLaunchArgument("enable_pose_debug", default_value="true"),
            DeclareLaunchArgument("debug_publish_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("debug_history_window_s", default_value="5.0"),
            DeclareLaunchArgument("debug_hold_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("debug_warn_gap_s", default_value="0.25"),
            DeclareLaunchArgument("debug_warn_position_error_m", default_value="0.10"),
            DeclareLaunchArgument("debug_warn_yaw_error_deg", default_value="10.0"),
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
                "local_pose_timeout_during_param_sync_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument("state_timeout_s", default_value="2.0"),
            DeclareLaunchArgument(
                "state_timeout_during_param_sync_s",
                default_value="8.0",
            ),
            DeclareLaunchArgument("connection_loss_timeout_s", default_value="0.5"),
            DeclareLaunchArgument(
                "connection_loss_timeout_during_param_sync_s",
                default_value="3.0",
            ),
            DeclareLaunchArgument("companion_status_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("require_companion_active", default_value="false"),
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
            DeclareLaunchArgument("restore_speed_profile_on_exit", default_value="true"),
            DeclareLaunchArgument(
                "allow_synthetic_home_position",
                default_value="true",
            ),
            DeclareLaunchArgument("require_global_origin", default_value="false"),
            DeclareLaunchArgument("synthetic_home_position_z_m", default_value="0.0"),
            DeclareLaunchArgument(
                "assume_global_origin_after_publish",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "assume_home_position_after_publish",
                default_value="true",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
