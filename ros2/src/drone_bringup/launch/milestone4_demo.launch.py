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


def _load_defaults() -> dict[str, object]:
    return load_drone_launch_defaults()


def _join_topic(namespace: str, leaf: str) -> str:
    namespace = str(namespace or "").strip()
    if namespace and not namespace.startswith("/"):
        namespace = "/" + namespace
    namespace = namespace.rstrip("/")
    leaf = "/" + str(leaf or "").strip().lstrip("/")
    return f"{namespace}{leaf}" if namespace else leaf


def _cdrone_topic(drone_namespace: str, drone_id: str, leaf: str) -> str:
    namespace = str(drone_namespace or "").strip().rstrip("/")
    drone_id = str(drone_id or "").strip().strip("/")
    if drone_id and not namespace.endswith(f"/{drone_id}"):
        namespace = _join_topic(namespace, drone_id).rstrip("/")
    return _join_topic(namespace, leaf)


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    drone_id_text = LaunchConfiguration("drone_id").perform(context).strip()
    drone_namespace_text = (
        LaunchConfiguration("drone_namespace").perform(context).strip() or "/cdrone"
    )
    target_map_tracks_topic = (
        LaunchConfiguration("target_map_tracks_topic").perform(context).strip()
        or _cdrone_topic(drone_namespace_text, drone_id_text, "target_map/tracks")
    )
    target_map_world_topic = (
        LaunchConfiguration("target_map_world_topic").perform(context).strip()
        or _cdrone_topic(drone_namespace_text, drone_id_text, "target_map/world_tracks")
    )
    target_map_markers_topic = (
        LaunchConfiguration("target_map_markers_topic").perform(context).strip()
        or _cdrone_topic(drone_namespace_text, drone_id_text, "target_map/markers")
    )
    target_map_input_world_tracks_topic = (
        LaunchConfiguration("target_map_input_world_tracks_topic")
        .perform(context)
        .strip()
        or _cdrone_topic(drone_namespace_text, drone_id_text, "perception/world_tracks")
    )
    tracking_pose_topic = (
        LaunchConfiguration("tracking_pose_topic").perform(context).strip()
        or _cdrone_topic(drone_namespace_text, drone_id_text, "external_pose/input_pose")
    )
    target_map_ownship_pose_topic = (
        LaunchConfiguration("target_map_ownship_pose_topic").perform(context).strip()
        or tracking_pose_topic
    )
    tracking_metrics_raw_world_topic = (
        LaunchConfiguration("tracking_metrics_raw_world_topic").perform(context).strip()
        or target_map_input_world_tracks_topic
    )
    tracking_metrics_target_map_world_topic = (
        LaunchConfiguration("tracking_metrics_target_map_world_topic")
        .perform(context)
        .strip()
        or target_map_world_topic
    )
    tracking_metrics_ownship_pose_topic = (
        LaunchConfiguration("tracking_metrics_ownship_pose_topic")
        .perform(context)
        .strip()
        or target_map_ownship_pose_topic
    )

    target_map_node = Node(
        package="drone_vision_pkg",
        executable="target_map_node",
        namespace=LaunchConfiguration("drone_namespace"),
        output="screen",
        parameters=[
            LaunchConfiguration("target_map_config_file"),
            {
                "drone_id": LaunchConfiguration("drone_id"),
                "mavros_namespace": LaunchConfiguration("mavros_namespace"),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("target_map_rate_hz"),
                    value_type=float,
                ),
                "world_frame": LaunchConfiguration("map_frame"),
                "world_tracks_topic": target_map_input_world_tracks_topic,
                "ownship_pose_topic": target_map_ownship_pose_topic,
                "target_map_world_topic": target_map_world_topic,
                "target_map_tracks_topic": target_map_tracks_topic,
                "target_map_markers_topic": target_map_markers_topic,
                "association_gate_m": ParameterValue(
                    LaunchConfiguration("target_map_association_gate_m"),
                    value_type=float,
                ),
                "max_prediction_horizon_s": ParameterValue(
                    LaunchConfiguration("target_map_prediction_horizon_s"),
                    value_type=float,
                ),
                "prune_after_s": ParameterValue(
                    LaunchConfiguration("target_map_prune_after_s"),
                    value_type=float,
                ),
                "confidence_decay_per_s": ParameterValue(
                    LaunchConfiguration("target_map_confidence_decay_per_s"),
                    value_type=float,
                ),
                "position_process_noise_mps": ParameterValue(
                    LaunchConfiguration("target_map_position_process_noise_mps"),
                    value_type=float,
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    milestone3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "milestone3_demo.launch.py")
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "drone_id": LaunchConfiguration("drone_id"),
            "drone_namespace": LaunchConfiguration("drone_namespace"),
            "mavros_namespace": LaunchConfiguration("mavros_namespace"),
            "mocap_namespace": LaunchConfiguration("mocap_namespace"),
            "pose_source": LaunchConfiguration("pose_source"),
            "source_pose_topic": LaunchConfiguration("source_pose_topic"),
            "tracks_topic": target_map_tracks_topic,
            "engagement_state_topic": LaunchConfiguration("engagement_state_topic"),
            "enable_predicted_track_control": "true",
            "predicted_track_hold_s": LaunchConfiguration("predicted_track_hold_s"),
            "max_predicted_position_uncertainty_m": LaunchConfiguration(
                "max_predicted_position_uncertainty_m"
            ),
            "predicted_max_vel_xy_mps": LaunchConfiguration(
                "predicted_max_vel_xy_mps"
            ),
            "predicted_max_vel_z_mps": LaunchConfiguration("predicted_max_vel_z_mps"),
            "predicted_max_yaw_rate_rps": LaunchConfiguration(
                "predicted_max_yaw_rate_rps"
            ),
            "enable_led_indicator": LaunchConfiguration("enable_led_indicator"),
            "led_indicator_dry_run": LaunchConfiguration("led_indicator_dry_run"),
            "led_indicator_fade_ms": LaunchConfiguration("led_indicator_fade_ms"),
            "tracking_source_mode": LaunchConfiguration("tracking_source_mode"),
            "zed_rotate_180": LaunchConfiguration("zed_rotate_180"),
            "tracking_pose_topic": tracking_pose_topic,
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
            "tracker_enable_reid": LaunchConfiguration("tracker_enable_reid"),
            "tracker_reid_histogram_bins": LaunchConfiguration(
                "tracker_reid_histogram_bins"
            ),
            "tracker_reid_distance_threshold": LaunchConfiguration(
                "tracker_reid_distance_threshold"
            ),
            "tracker_reid_hit_counter_max": LaunchConfiguration(
                "tracker_reid_hit_counter_max"
            ),
            "tracker_enable_motion_estimator": LaunchConfiguration(
                "tracker_enable_motion_estimator"
            ),
            "tracker_publish_track_hold_s": LaunchConfiguration(
                "tracker_publish_track_hold_s"
            ),
            "tracker_publish_track_hold_max_extrapolation_m": LaunchConfiguration(
                "tracker_publish_track_hold_max_extrapolation_m"
            ),
            "recording_target_map_world_topic": target_map_world_topic,
            "recording_save_depth_video": "false",
            "recording_replace_depth_tile_with_yolo": "true",
            "enable_tracking_metrics": LaunchConfiguration("enable_tracking_metrics"),
            "tracking_metrics_output_dir": LaunchConfiguration(
                "tracking_metrics_output_dir"
            ),
            "tracking_metrics_experiment_tag": LaunchConfiguration(
                "tracking_metrics_experiment_tag"
            ),
            "tracking_metrics_raw_world_topic": tracking_metrics_raw_world_topic,
            "tracking_metrics_target_map_world_topic": (
                tracking_metrics_target_map_world_topic
            ),
            "tracking_metrics_ownship_pose_topic": tracking_metrics_ownship_pose_topic,
            "tracking_metrics_engagement_state_topic": LaunchConfiguration(
                "engagement_state_topic"
            ),
            "tracking_metrics_rate_hz": LaunchConfiguration(
                "tracking_metrics_rate_hz"
            ),
            "tracking_metrics_track_stale_s": LaunchConfiguration(
                "tracking_metrics_track_stale_s"
            ),
            "tracking_metrics_max_reacquisition_gap_s": LaunchConfiguration(
                "tracking_metrics_max_reacquisition_gap_s"
            ),
            "tracking_metrics_simulated_dropout_horizon_s": LaunchConfiguration(
                "tracking_metrics_simulated_dropout_horizon_s"
            ),
            "tracking_metrics_path_smoothing_window": LaunchConfiguration(
                "tracking_metrics_path_smoothing_window"
            ),
            "tracking_metrics_write_video": LaunchConfiguration(
                "tracking_metrics_write_video"
            ),
            "tracking_metrics_video_fps": LaunchConfiguration(
                "tracking_metrics_video_fps"
            ),
            "tracking_metrics_video_width": LaunchConfiguration(
                "tracking_metrics_video_width"
            ),
            "tracking_metrics_video_height": LaunchConfiguration(
                "tracking_metrics_video_height"
            ),
            "tracking_metrics_video_fourcc": LaunchConfiguration(
                "tracking_metrics_video_fourcc"
            ),
            "tracking_metrics_postprocess_cleaned_tracks": LaunchConfiguration(
                "tracking_metrics_postprocess_cleaned_tracks"
            ),
            "tracking_metrics_reference_video": LaunchConfiguration(
                "tracking_metrics_reference_video"
            ),
            "tracking_metrics_reference_recording_dir": LaunchConfiguration(
                "tracking_metrics_reference_recording_dir"
            ),
            "tracking_metrics_reference_video_wait_s": LaunchConfiguration(
                "tracking_metrics_reference_video_wait_s"
            ),
            "demo_config": LaunchConfiguration("demo_config"),
            "required_completion_count": LaunchConfiguration(
                "required_completion_count"
            ),
            "takeoff_altitude_m": LaunchConfiguration("takeoff_altitude_m"),
            "min_target_distance_m": LaunchConfiguration("min_target_distance_m"),
            "max_target_distance_m": LaunchConfiguration("max_target_distance_m"),
            "perimeter_config": LaunchConfiguration("perimeter_config"),
            "pre_takeoff_profile_config": LaunchConfiguration(
                "pre_takeoff_profile_config"
            ),
            "use_speed_profile": LaunchConfiguration("use_speed_profile"),
            "speed_profile": LaunchConfiguration("speed_profile"),
            "speed_profile_config": LaunchConfiguration("speed_profile_config"),
            "map_frame": LaunchConfiguration("map_frame"),
            "frame_rpy_rad": LaunchConfiguration("frame_rpy_rad"),
            "rpy_offset_rad": LaunchConfiguration("rpy_offset_rad"),
            "optitrack_server": LaunchConfiguration("optitrack_server"),
            "optitrack_port": LaunchConfiguration("optitrack_port"),
            "rigid_body_name": LaunchConfiguration("rigid_body_name"),
            "velocity_rate_hz": LaunchConfiguration("velocity_rate_hz"),
            "watchdog_timeout_s": LaunchConfiguration("watchdog_timeout_s"),
            "require_guided_mode": LaunchConfiguration("require_guided_mode"),
            "follow_rate_hz": LaunchConfiguration("follow_rate_hz"),
        }.items(),
    )

    return [milestone3_launch, target_map_node]


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    vision_share = get_package_share_directory("drone_vision_pkg")
    defaults = _load_defaults()
    default_tracker_config = os.path.join(
        vision_share,
        "config",
        "zed2i_tracking.yaml",
    )
    default_target_map_config = os.path.join(
        vision_share,
        "config",
        "target_map.yaml",
    )
    default_demo_config = os.path.join(
        bringup_share,
        "config",
        "milestone4_demo.yaml",
    )
    default_perimeter_config = os.path.join(
        bringup_share,
        "config",
        "drone_studio_perimeter.yaml",
    )
    default_pre_takeoff_profile_config = os.path.join(
        bringup_share,
        "config",
        "milestone3_takeoff_baseline_profile.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "drone_id",
                default_value=_default_arg(defaults, "drone_id", ""),
            ),
            DeclareLaunchArgument(
                "drone_namespace",
                default_value=_default_arg(defaults, "drone_namespace", "/cdrone"),
            ),
            DeclareLaunchArgument(
                "mavros_namespace",
                default_value=_default_arg(defaults, "mavros_namespace", "mavros"),
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
                "tracking_pose_topic",
                default_value="",
            ),
            DeclareLaunchArgument("target_map_ownship_pose_topic", default_value=""),
            DeclareLaunchArgument("target_map_input_world_tracks_topic", default_value=""),
            DeclareLaunchArgument("target_map_world_topic", default_value=""),
            DeclareLaunchArgument("target_map_tracks_topic", default_value=""),
            DeclareLaunchArgument("target_map_markers_topic", default_value=""),
            DeclareLaunchArgument(
                "target_map_config_file",
                default_value=default_target_map_config,
            ),
            DeclareLaunchArgument("target_map_rate_hz", default_value="10.0"),
            DeclareLaunchArgument(
                "target_map_association_gate_m",
                default_value="0.8",
            ),
            DeclareLaunchArgument(
                "target_map_prediction_horizon_s",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "target_map_prune_after_s",
                default_value="5.0",
            ),
            DeclareLaunchArgument(
                "target_map_confidence_decay_per_s",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "target_map_position_process_noise_mps",
                default_value="0.35",
            ),
            DeclareLaunchArgument("predicted_track_hold_s", default_value="1.5"),
            DeclareLaunchArgument(
                "max_predicted_position_uncertainty_m",
                default_value="0.75",
            ),
            DeclareLaunchArgument("predicted_max_vel_xy_mps", default_value="0.25"),
            DeclareLaunchArgument("predicted_max_vel_z_mps", default_value="0.15"),
            DeclareLaunchArgument("predicted_max_yaw_rate_rps", default_value="0.25"),
            DeclareLaunchArgument("engagement_state_topic", default_value=""),
            DeclareLaunchArgument("enable_led_indicator", default_value="true"),
            DeclareLaunchArgument("led_indicator_dry_run", default_value="false"),
            DeclareLaunchArgument("led_indicator_fade_ms", default_value="250"),
            DeclareLaunchArgument("tracking_source_mode", default_value="zed_sdk"),
            DeclareLaunchArgument("zed_rotate_180", default_value="true"),
            DeclareLaunchArgument(
                "publish_world_track_compare",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "compare_pose_topic",
                default_value=_default_arg(defaults, "compare_pose_topic", ""),
            ),
            DeclareLaunchArgument("world_track_compare_topic", default_value=""),
            DeclareLaunchArgument("experiment_tag", default_value="milestone4"),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=default_tracker_config,
            ),
            DeclareLaunchArgument("tracker_enable_reid", default_value="true"),
            DeclareLaunchArgument("tracker_reid_histogram_bins", default_value="32"),
            DeclareLaunchArgument(
                "tracker_reid_distance_threshold",
                default_value="0.20",
            ),
            DeclareLaunchArgument("tracker_reid_hit_counter_max", default_value="30"),
            DeclareLaunchArgument(
                "tracker_enable_motion_estimator",
                default_value="true",
            ),
            DeclareLaunchArgument("tracker_publish_track_hold_s", default_value="0.5"),
            DeclareLaunchArgument(
                "tracker_publish_track_hold_max_extrapolation_m",
                default_value="0.25",
            ),
            DeclareLaunchArgument("enable_tracking_metrics", default_value="true"),
            DeclareLaunchArgument(
                "tracking_metrics_output_dir",
                default_value=(
                    "/home/jetson/cdrone_control/mission_recordings/"
                    "tracking_metrics"
                ),
            ),
            DeclareLaunchArgument(
                "tracking_metrics_experiment_tag",
                default_value="milestone4",
            ),
            DeclareLaunchArgument("tracking_metrics_raw_world_topic", default_value=""),
            DeclareLaunchArgument(
                "tracking_metrics_target_map_world_topic",
                default_value="",
            ),
            DeclareLaunchArgument(
                "tracking_metrics_ownship_pose_topic",
                default_value="",
            ),
            DeclareLaunchArgument("tracking_metrics_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("tracking_metrics_track_stale_s", default_value="0.5"),
            DeclareLaunchArgument(
                "tracking_metrics_max_reacquisition_gap_s",
                default_value="3.0",
            ),
            DeclareLaunchArgument(
                "tracking_metrics_simulated_dropout_horizon_s",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "tracking_metrics_path_smoothing_window",
                default_value="5",
            ),
            DeclareLaunchArgument("tracking_metrics_write_video", default_value="true"),
            DeclareLaunchArgument("tracking_metrics_video_fps", default_value="10.0"),
            DeclareLaunchArgument("tracking_metrics_video_width", default_value="1280"),
            DeclareLaunchArgument("tracking_metrics_video_height", default_value="720"),
            DeclareLaunchArgument("tracking_metrics_video_fourcc", default_value="mp4v"),
            DeclareLaunchArgument(
                "tracking_metrics_postprocess_cleaned_tracks",
                default_value="true",
            ),
            DeclareLaunchArgument("tracking_metrics_reference_video", default_value=""),
            DeclareLaunchArgument(
                "tracking_metrics_reference_recording_dir",
                default_value="/home/jetson/cdrone_control/mission_recordings",
            ),
            DeclareLaunchArgument(
                "tracking_metrics_reference_video_wait_s",
                default_value="5.0",
            ),
            DeclareLaunchArgument("demo_config", default_value=default_demo_config),
            DeclareLaunchArgument("required_completion_count", default_value="1"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="2.0"),
            DeclareLaunchArgument("min_target_distance_m", default_value="0.8"),
            DeclareLaunchArgument("max_target_distance_m", default_value="25.0"),
            DeclareLaunchArgument(
                "perimeter_config",
                default_value=default_perimeter_config,
            ),
            DeclareLaunchArgument(
                "pre_takeoff_profile_config",
                default_value=default_pre_takeoff_profile_config,
            ),
            DeclareLaunchArgument("use_speed_profile", default_value="true"),
            DeclareLaunchArgument("speed_profile", default_value="indoor"),
            DeclareLaunchArgument("speed_profile_config", default_value=""),
            DeclareLaunchArgument(
                "map_frame",
                default_value=_default_arg(defaults, "map_frame", "map"),
            ),
            DeclareLaunchArgument(
                "frame_rpy_rad",
                default_value="[0.0, 0.0, 3.141592653589793]",
            ),
            DeclareLaunchArgument(
                "rpy_offset_rad", default_value="[0.0, 0.0, 0.0]"
            ),
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
            DeclareLaunchArgument("velocity_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("watchdog_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("require_guided_mode", default_value="true"),
            DeclareLaunchArgument("follow_rate_hz", default_value="20.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
