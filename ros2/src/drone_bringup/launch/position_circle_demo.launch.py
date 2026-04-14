import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")

    goto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "position_goto_demo.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "drone_id": LaunchConfiguration("drone_id"),
            "mavros_namespace": LaunchConfiguration("mavros_namespace"),
            "pose_source": LaunchConfiguration("pose_source"),
            "source_pose_topic": LaunchConfiguration("source_pose_topic"),
            "demo_mode": "circle",
            "takeoff_altitude_m": LaunchConfiguration("takeoff_altitude_m"),
            "takeoff_rate_m_s": LaunchConfiguration("takeoff_rate_m_s"),
            "takeoff_strategy": LaunchConfiguration("takeoff_strategy"),
            "altitude_tolerance_m": LaunchConfiguration("altitude_tolerance_m"),
            "touchdown_altitude_m": LaunchConfiguration("touchdown_altitude_m"),
            "touchdown_dwell_s": LaunchConfiguration("touchdown_dwell_s"),
            "stage_timeout_s": LaunchConfiguration("stage_timeout_s"),
            "arm_zero_throttle_hold_s": LaunchConfiguration("arm_zero_throttle_hold_s"),
            "local_pose_timeout_s": LaunchConfiguration("local_pose_timeout_s"),
            "local_pose_timeout_during_param_sync_s": LaunchConfiguration(
                "local_pose_timeout_during_param_sync_s"
            ),
            "state_timeout_s": LaunchConfiguration("state_timeout_s"),
            "state_timeout_during_param_sync_s": LaunchConfiguration(
                "state_timeout_during_param_sync_s"
            ),
            "connection_loss_timeout_s": LaunchConfiguration("connection_loss_timeout_s"),
            "connection_loss_timeout_during_param_sync_s": LaunchConfiguration(
                "connection_loss_timeout_during_param_sync_s"
            ),
            "companion_status_timeout_s": LaunchConfiguration("companion_status_timeout_s"),
            "require_companion_active": LaunchConfiguration("require_companion_active"),
            "restore_takeoff_alt_on_exit": LaunchConfiguration("restore_takeoff_alt_on_exit"),
            "takeoff_param_id": LaunchConfiguration("takeoff_param_id"),
            "param_pull_force": LaunchConfiguration("param_pull_force"),
            "param_pull_retry_delay_s": LaunchConfiguration("param_pull_retry_delay_s"),
            "param_sync_timeout_s": LaunchConfiguration("param_sync_timeout_s"),
            "use_speed_profile": LaunchConfiguration("use_speed_profile"),
            "speed_profile_config": LaunchConfiguration("speed_profile_config"),
            "restore_speed_profile_on_exit": LaunchConfiguration(
                "restore_speed_profile_on_exit"
            ),
            "offboard_setpoint_warmup_s": LaunchConfiguration("offboard_setpoint_warmup_s"),
            "goal_frame_id": LaunchConfiguration("goal_frame_id"),
            "goal_position_tolerance_m": LaunchConfiguration("goal_position_tolerance_m"),
            "goal_hold_duration_s": LaunchConfiguration("goal_hold_duration_s"),
            "circle_center_x_m": LaunchConfiguration("circle_center_x_m"),
            "circle_center_y_m": LaunchConfiguration("circle_center_y_m"),
            "circle_radius_m": LaunchConfiguration("circle_radius_m"),
            "circle_altitude_m": LaunchConfiguration("circle_altitude_m"),
            "circle_speed_mps": LaunchConfiguration("circle_speed_mps"),
            "circle_loops": LaunchConfiguration("circle_loops"),
            "circle_clockwise": LaunchConfiguration("circle_clockwise"),
            "circle_use_keep_out_orbit": LaunchConfiguration(
                "circle_use_keep_out_orbit"
            ),
            "circle_keep_out_name": LaunchConfiguration("circle_keep_out_name"),
            "circle_keep_out_clearance_m": LaunchConfiguration(
                "circle_keep_out_clearance_m"
            ),
            "circle_sample_count": LaunchConfiguration("circle_sample_count"),
            "circle_entry_candidate_count": LaunchConfiguration(
                "circle_entry_candidate_count"
            ),
            "max_circle_entry_distance_from_start_m": LaunchConfiguration(
                "max_circle_entry_distance_from_start_m"
            ),
            "enable_perimeter_guard": LaunchConfiguration("enable_perimeter_guard"),
            "perimeter_config": LaunchConfiguration("perimeter_config"),
            "perimeter_segment_sample_step_m": LaunchConfiguration(
                "perimeter_segment_sample_step_m"
            ),
            "perimeter_boundary_tolerance_m": LaunchConfiguration(
                "perimeter_boundary_tolerance_m"
            ),
            "perimeter_boundary_margin_m": LaunchConfiguration(
                "perimeter_boundary_margin_m"
            ),
            "perimeter_keep_out_margin_m": LaunchConfiguration(
                "perimeter_keep_out_margin_m"
            ),
            "perimeter_ceiling_tolerance_m": LaunchConfiguration(
                "perimeter_ceiling_tolerance_m"
            ),
            "global_origin_latitude_deg": LaunchConfiguration("global_origin_latitude_deg"),
            "global_origin_longitude_deg": LaunchConfiguration(
                "global_origin_longitude_deg"
            ),
            "global_origin_altitude_m": LaunchConfiguration("global_origin_altitude_m"),
            "home_position_x_m": LaunchConfiguration("home_position_x_m"),
            "home_position_y_m": LaunchConfiguration("home_position_y_m"),
            "home_position_z_m": LaunchConfiguration("home_position_z_m"),
            "home_approach_z_m": LaunchConfiguration("home_approach_z_m"),
            "reference_retry_period_s": LaunchConfiguration("reference_retry_period_s"),
            "external_pose_publish_rate_hz": LaunchConfiguration(
                "external_pose_publish_rate_hz"
            ),
            "external_pose_timeout_s": LaunchConfiguration("external_pose_timeout_s"),
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

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            DeclareLaunchArgument("pose_source", default_value="optitrack"),
            DeclareLaunchArgument("source_pose_topic", default_value=""),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="0.7"),
            DeclareLaunchArgument("takeoff_rate_m_s", default_value="0.5"),
            DeclareLaunchArgument("takeoff_strategy", default_value="AUTO_MODE"),
            DeclareLaunchArgument("altitude_tolerance_m", default_value="0.10"),
            DeclareLaunchArgument("touchdown_altitude_m", default_value="0.15"),
            DeclareLaunchArgument("touchdown_dwell_s", default_value="1.0"),
            DeclareLaunchArgument("stage_timeout_s", default_value="30.0"),
            DeclareLaunchArgument("arm_zero_throttle_hold_s", default_value="1.5"),
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
            DeclareLaunchArgument("require_companion_active", default_value="true"),
            DeclareLaunchArgument("restore_takeoff_alt_on_exit", default_value="true"),
            DeclareLaunchArgument("takeoff_param_id", default_value="MIS_TAKEOFF_ALT"),
            DeclareLaunchArgument("param_pull_force", default_value="true"),
            DeclareLaunchArgument("param_pull_retry_delay_s", default_value="1.0"),
            DeclareLaunchArgument("param_sync_timeout_s", default_value="60.0"),
            DeclareLaunchArgument("use_speed_profile", default_value="false"),
            DeclareLaunchArgument(
                "speed_profile_config",
                default_value=os.path.join(
                    bringup_share,
                    "config",
                    "indoor_speed_profile.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "restore_speed_profile_on_exit",
                default_value="true",
            ),
            DeclareLaunchArgument("offboard_setpoint_warmup_s", default_value="1.5"),
            DeclareLaunchArgument("goal_frame_id", default_value="map"),
            DeclareLaunchArgument("goal_position_tolerance_m", default_value="0.20"),
            DeclareLaunchArgument("goal_hold_duration_s", default_value="2.0"),
            DeclareLaunchArgument("circle_center_x_m", default_value="0.0"),
            DeclareLaunchArgument("circle_center_y_m", default_value="0.0"),
            DeclareLaunchArgument("circle_radius_m", default_value="0.0"),
            DeclareLaunchArgument("circle_altitude_m", default_value="2.0"),
            DeclareLaunchArgument("circle_speed_mps", default_value="0.5"),
            DeclareLaunchArgument("circle_loops", default_value="1.0"),
            DeclareLaunchArgument("circle_clockwise", default_value="false"),
            DeclareLaunchArgument("circle_use_keep_out_orbit", default_value="true"),
            DeclareLaunchArgument(
                "circle_keep_out_name",
                default_value="studio_pillar",
            ),
            DeclareLaunchArgument(
                "circle_keep_out_clearance_m",
                default_value="1.2",
            ),
            DeclareLaunchArgument("circle_sample_count", default_value="180"),
            DeclareLaunchArgument("circle_entry_candidate_count", default_value="72"),
            DeclareLaunchArgument(
                "max_circle_entry_distance_from_start_m",
                default_value="8.0",
            ),
            DeclareLaunchArgument("enable_perimeter_guard", default_value="true"),
            DeclareLaunchArgument(
                "perimeter_config",
                default_value=os.path.join(
                    bringup_share,
                    "config",
                    "drone_studio_perimeter.yaml",
                ),
            ),
            DeclareLaunchArgument(
                "perimeter_segment_sample_step_m",
                default_value="0.10",
            ),
            DeclareLaunchArgument(
                "perimeter_boundary_tolerance_m",
                default_value="0.05",
            ),
            DeclareLaunchArgument(
                "perimeter_boundary_margin_m",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "perimeter_keep_out_margin_m",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "perimeter_ceiling_tolerance_m",
                default_value="0.05",
            ),
            DeclareLaunchArgument("global_origin_latitude_deg", default_value="0.0"),
            DeclareLaunchArgument("global_origin_longitude_deg", default_value="0.0"),
            DeclareLaunchArgument("global_origin_altitude_m", default_value="17.1637"),
            DeclareLaunchArgument("home_position_x_m", default_value="0.0"),
            DeclareLaunchArgument("home_position_y_m", default_value="0.0"),
            DeclareLaunchArgument("home_position_z_m", default_value="0.0"),
            DeclareLaunchArgument("home_approach_z_m", default_value="1.0"),
            DeclareLaunchArgument("reference_retry_period_s", default_value="1.0"),
            DeclareLaunchArgument("external_pose_publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("external_pose_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("position_offset_m", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("rpy_offset_rad", default_value="[0.0, 0.0, 0.0]"),
            DeclareLaunchArgument("optitrack_server", default_value="192.168.0.217"),
            DeclareLaunchArgument("optitrack_port", default_value="3883"),
            DeclareLaunchArgument("rigid_body_name", default_value="RigidBody3"),
            DeclareLaunchArgument("vrpn_update_freq", default_value="100.0"),
            DeclareLaunchArgument("vrpn_refresh_freq", default_value="1.0"),
            DeclareLaunchArgument("vrpn_sensor_data_qos", default_value="true"),
            DeclareLaunchArgument("source_best_effort", default_value="true"),
            DeclareLaunchArgument("use_vrpn_timestamps", default_value="false"),
            DeclareLaunchArgument("enable_pose_debug", default_value="true"),
            DeclareLaunchArgument("debug_publish_rate_hz", default_value="1.0"),
            DeclareLaunchArgument("debug_history_window_s", default_value="5.0"),
            DeclareLaunchArgument("debug_hold_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("debug_warn_gap_s", default_value="0.25"),
            DeclareLaunchArgument("debug_warn_position_error_m", default_value="0.10"),
            DeclareLaunchArgument("debug_warn_yaw_error_deg", default_value="10.0"),
            goto_launch,
        ]
    )
