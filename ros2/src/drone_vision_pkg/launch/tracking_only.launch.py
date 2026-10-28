from pathlib import Path

from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    vision_share = Path(__file__).resolve().parents[1]
    default_tracker_config = str(vision_share / "config" / "tracking_only.yaml")
    defaults = load_drone_launch_defaults()

    tracker_parameters = [
            LaunchConfiguration("tracker_config_file"),
            {
                "drone_id": LaunchConfiguration("drone_id"),
                "source_mode": LaunchConfiguration("source_mode"),
                "zed_rotate_180": ParameterValue(
                    LaunchConfiguration("zed_rotate_180"), value_type=bool),
                "zed_mount_config": LaunchConfiguration("zed_mount_config"),
                "zed_ready_before_inference_s": ParameterValue(
                    LaunchConfiguration("zed_ready_before_inference_s"),
                    value_type=float),
                "inference_device": ParameterValue(
                    LaunchConfiguration("inference_device"), value_type=str),
                "color_topic": LaunchConfiguration("color_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "pose_topic": LaunchConfiguration("pose_topic"),
                "publish_world_track_compare": LaunchConfiguration(
                    "publish_world_track_compare"
                ),
                "compare_pose_topic": LaunchConfiguration("compare_pose_topic"),
                "world_track_compare_topic": LaunchConfiguration(
                    "world_track_compare_topic"
                ),
                "experiment_tag": LaunchConfiguration("experiment_tag"),
                "model_path": LaunchConfiguration("model_path"),
                "model_task": LaunchConfiguration("model_task"),
                "model_data_config": LaunchConfiguration("model_data_config"),
                "record_mission_video": ParameterValue(
                    LaunchConfiguration("record_mission_video"), value_type=bool),
                "enable_reid": LaunchConfiguration("enable_reid"),
                "reid_histogram_bins": LaunchConfiguration("reid_histogram_bins"),
                "reid_distance_threshold": LaunchConfiguration(
                    "reid_distance_threshold"
                ),
                "reid_hit_counter_max": LaunchConfiguration("reid_hit_counter_max"),
                "enable_motion_estimator": LaunchConfiguration(
                    "enable_motion_estimator"
                ),
                "publish_track_hold_s": LaunchConfiguration("publish_track_hold_s"),
                "publish_track_hold_max_extrapolation_m": LaunchConfiguration(
                    "publish_track_hold_max_extrapolation_m"
                ),
                "recording_target_map_world_topic": LaunchConfiguration(
                    "recording_target_map_world_topic"
                ),
                "recording_save_depth_video": LaunchConfiguration(
                    "recording_save_depth_video"
                ),
                "recording_replace_depth_tile_with_yolo": LaunchConfiguration(
                    "recording_replace_depth_tile_with_yolo"
                ),
            },
        ]
    tracker_node = Node(
        package="drone_vision_pkg",
        executable="realsense_tracker_node",
        namespace=LaunchConfiguration("drone_namespace"),
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("embedded_zed")),
        parameters=tracker_parameters,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    combined_node = Node(
        package="drone_vision_pkg",
        executable="zed_local_combined_node",
        namespace=LaunchConfiguration("drone_namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("embedded_zed")),
        parameters=tracker_parameters + [{
            "mavros_namespace": LaunchConfiguration("mavros_namespace"),
            "mount_config": LaunchConfiguration("zed_mount_config"),
            # The embedded tracker never opens a second SDK camera, but keep
            # its diagnostic parameter consistent with the active VIO camera.
            "zed_depth_mode": LaunchConfiguration("zed_depth_mode"),
            "capture_frames": True,
            "publish_images": False,
            "rotate_180": ParameterValue(
                LaunchConfiguration("zed_rotate_180"), value_type=bool),
            "depth_mode": LaunchConfiguration("zed_depth_mode"),
            "image_rate_hz": ParameterValue(
                LaunchConfiguration("zed_image_rate_hz"), value_type=float),
            "grab_compute_capping_fps": ParameterValue(
                LaunchConfiguration("zed_grab_compute_capping_fps"), value_type=float),
            "depth_maximum_distance_m": ParameterValue(
                LaunchConfiguration("zed_depth_maximum_distance_m"), value_type=float),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "drone_id",
                default_value=default_arg(defaults, "drone_id", ""),
            ),
            DeclareLaunchArgument(
                "drone_namespace",
                default_value=default_arg(defaults, "drone_namespace", "/cdrone"),
            ),
            DeclareLaunchArgument("source_mode", default_value="direct"),
            DeclareLaunchArgument("embedded_zed", default_value="false"),
            DeclareLaunchArgument("zed_mount_config", default_value=""),
            DeclareLaunchArgument("mavros_namespace", default_value="/cdrone/cdrone3/mavros"),
            DeclareLaunchArgument("zed_depth_mode", default_value="PERFORMANCE"),
            DeclareLaunchArgument("zed_image_rate_hz", default_value="8.0"),
            DeclareLaunchArgument("zed_grab_compute_capping_fps", default_value="0.0"),
            DeclareLaunchArgument("zed_depth_maximum_distance_m", default_value="20.0"),
            DeclareLaunchArgument("zed_ready_before_inference_s", default_value="0.0"),
            DeclareLaunchArgument("inference_device", default_value=""),
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
            DeclareLaunchArgument("zed_rotate_180", default_value="true"),
            DeclareLaunchArgument(
                "tracker_config_file",
                default_value=default_tracker_config,
            ),
            DeclareLaunchArgument(
                "pose_topic",
                default_value=default_arg(defaults, "ownship_pose_topic", ""),
            ),
            DeclareLaunchArgument(
                "publish_world_track_compare", default_value="true"
            ),
            DeclareLaunchArgument(
                "compare_pose_topic",
                default_value=default_arg(defaults, "compare_pose_topic", ""),
            ),
            DeclareLaunchArgument("world_track_compare_topic", default_value=""),
            DeclareLaunchArgument("experiment_tag", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("model_task", default_value="detect"),
            DeclareLaunchArgument("model_data_config", default_value=""),
            DeclareLaunchArgument("record_mission_video", default_value="true"),
            DeclareLaunchArgument("enable_reid", default_value="false"),
            DeclareLaunchArgument("reid_histogram_bins", default_value="32"),
            DeclareLaunchArgument(
                "reid_distance_threshold",
                default_value="0.20",
            ),
            DeclareLaunchArgument("reid_hit_counter_max", default_value="30"),
            DeclareLaunchArgument(
                "enable_motion_estimator",
                default_value="false",
            ),
            DeclareLaunchArgument("publish_track_hold_s", default_value="0.0"),
            DeclareLaunchArgument(
                "publish_track_hold_max_extrapolation_m",
                default_value="0.25",
            ),
            DeclareLaunchArgument(
                "recording_target_map_world_topic",
                default_value="",
            ),
            DeclareLaunchArgument("recording_save_depth_video", default_value="true"),
            DeclareLaunchArgument(
                "recording_replace_depth_tile_with_yolo",
                default_value="false",
            ),
            tracker_node,
            combined_node,
        ]
    )
