"""One-terminal, separate-process MAVROS/ZED/detector diagnostics."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from drone_control_pkg.deployment_config import default_arg, load_drone_launch_defaults
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    stage = LaunchConfiguration("stage").perform(context).strip().lower()
    if stage not in {"mavros", "zed", "all"}:
        raise ValueError("stage must be mavros, zed, or all")
    zed_delay_s = float(LaunchConfiguration("zed_start_delay_s").perform(context))
    detector_delay_s = float(LaunchConfiguration("detector_start_delay_s").perform(context))
    if not (0.0 <= zed_delay_s <= 60.0 and
            zed_delay_s <= detector_delay_s <= 90.0):
        raise ValueError("diagnostic start delays must satisfy 0 <= ZED <= detector <= 90 s")
    model_path = LaunchConfiguration("model_path").perform(context).strip()
    if stage == "all" and (Path(model_path).suffix.lower() != ".engine"
                           or not Path(model_path).is_file()):
        raise ValueError(f"Diagnostics require an existing .engine model: {model_path}")

    bringup_share = get_package_share_directory("drone_bringup")
    vision_share = get_package_share_directory("drone_vision_pkg")
    drone_id = LaunchConfiguration("drone_id")
    drone_namespace = LaunchConfiguration("drone_namespace")
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    prefix = "/cdrone/" + LaunchConfiguration("drone_id").perform(context).strip().strip("/")
    mavros_ns_text = LaunchConfiguration("mavros_namespace").perform(context).strip()
    pose_topic = "/" + mavros_ns_text.strip("/") + "/local_position/pose"
    actions = [LogInfo(msg=f"Milestone 4 diagnostics: stage={stage}; no mission controller")]
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "drone.launch.py")),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "mavros_namespace": mavros_namespace,
            "mavros_config_overlay": os.path.join(bringup_share, "config", "zed_mavros.yaml"),
        }.items(),
    ))
    if stage in {"zed", "all"}:
        zed_node = Node(
            package="drone_control_pkg", executable="zed_vio_node",
            namespace=drone_namespace, output="screen",
            parameters=[{
                "drone_id": drone_id,
                "mavros_namespace": mavros_namespace,
                "mount_config": LaunchConfiguration("zed_mount_config"),
                "publish_images": True,
                "rotate_180": True,
                "depth_mode": LaunchConfiguration("zed_depth_mode"),
                "image_rate_hz": ParameterValue(
                    LaunchConfiguration("zed_image_rate_hz"), value_type=float),
                "grab_compute_capping_fps": ParameterValue(
                    LaunchConfiguration("zed_grab_compute_capping_fps"), value_type=float),
                "depth_maximum_distance_m": 25.0,
            }],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
        actions.append(TimerAction(period=zed_delay_s, actions=[zed_node])
                       if zed_delay_s else zed_node)
    if stage == "all":
        tracker_node = Node(
            package="drone_vision_pkg", executable="realsense_tracker_node",
            namespace=drone_namespace, output="screen",
            parameters=[
                os.path.join(vision_share, "config", "zed2i_tracking.yaml"),
                {
                    "drone_id": drone_id,
                    "source_mode": "ros",
                    "zed_depth_mode": LaunchConfiguration("zed_depth_mode"),
                    "model_path": model_path,
                    "model_task": "obb",
                    "model_data_config": os.path.join(
                        vision_share, "config", "drone_detector.yaml"),
                    "inference_device": ParameterValue("0", value_type=str),
                    "zed_mount_config": LaunchConfiguration("zed_mount_config"),
                    "zed_rotate_180": True,
                    "zed_ready_before_inference_s": 5.0,
                    "color_topic": prefix + "/zed/color/image_rect",
                    "depth_topic": prefix + "/zed/depth/image_rect",
                    "camera_info_topic": prefix + "/zed/color/camera_info",
                    "pose_topic": pose_topic,
                    "publish_world_track_compare": False,
                    "save_world_track_compare_frames": False,
                    "save_world_track_compare_csv": False,
                    "save_world_track_compare_frame_csv": False,
                    "record_mission_video": False,
                    "enable_reid": False,
                    "enable_motion_estimator": False,
                },
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        )
        actions.append(TimerAction(period=detector_delay_s, actions=[tracker_node])
                       if detector_delay_s else tracker_node)
    return actions


def generate_launch_description():
    defaults = load_drone_launch_defaults()
    return LaunchDescription([
        DeclareLaunchArgument("stage", default_value="all"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("drone_id", default_value=default_arg(defaults, "drone_id", "cdrone3")),
        DeclareLaunchArgument("drone_namespace", default_value=default_arg(
            defaults, "drone_namespace", "/cdrone")),
        DeclareLaunchArgument("mavros_namespace", default_value=default_arg(
            defaults, "mavros_namespace", "/cdrone/cdrone3/mavros")),
        DeclareLaunchArgument("zed_mount_config", default_value="/home/jetson/zed_hover_mount_run2.yaml"),
        DeclareLaunchArgument("zed_depth_mode", default_value="PERFORMANCE"),
        DeclareLaunchArgument("zed_image_rate_hz", default_value="8.0"),
        DeclareLaunchArgument("zed_grab_compute_capping_fps", default_value="20.0"),
        DeclareLaunchArgument("zed_start_delay_s", default_value="15.0"),
        DeclareLaunchArgument("detector_start_delay_s", default_value="30.0"),
        DeclareLaunchArgument("model_path", default_value="/home/jetson/cdrone_control/models/best.engine"),
        OpaqueFunction(function=launch_setup),
    ])
