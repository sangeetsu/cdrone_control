import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def launch_setup(context, *args, **kwargs):
    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    drone_id = LaunchConfiguration("drone_id")

    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drone.launch.py")
        ),
        launch_arguments={
            "log_level": log_level,
            "mavros_namespace": mavros_namespace,
        }.items(),
    )

    bench_vision_pose_node = Node(
        package="drone_control_pkg",
        executable="bench_vision_pose_node",
        output="screen",
        parameters=[
            {
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("vision_rate_hz"), value_type=float
                ),
                "frame_id": LaunchConfiguration("vision_frame_id"),
                "x_m": ParameterValue(LaunchConfiguration("vision_x_m"), value_type=float),
                "y_m": ParameterValue(LaunchConfiguration("vision_y_m"), value_type=float),
                "z_m": ParameterValue(LaunchConfiguration("vision_z_m"), value_type=float),
                "roll_rad": ParameterValue(
                    LaunchConfiguration("vision_roll_rad"), value_type=float
                ),
                "pitch_rad": ParameterValue(
                    LaunchConfiguration("vision_pitch_rad"), value_type=float
                ),
                "yaw_rad": ParameterValue(
                    LaunchConfiguration("vision_yaw_rad"), value_type=float
                ),
                "publish_companion_status": ParameterValue(
                    LaunchConfiguration("publish_companion_status"), value_type=bool
                ),
                "mavros_namespace": mavros_namespace,
            }
        ],
        arguments=["--ros-args", "--log-level", log_level],
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

    return [drone_launch, bench_vision_pose_node, mavros_velocity_node]



def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            DeclareLaunchArgument("vision_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("vision_frame_id", default_value="map"),
            DeclareLaunchArgument("vision_x_m", default_value="0.0"),
            DeclareLaunchArgument("vision_y_m", default_value="0.0"),
            DeclareLaunchArgument("vision_z_m", default_value="0.0"),
            DeclareLaunchArgument("vision_roll_rad", default_value="0.0"),
            DeclareLaunchArgument("vision_pitch_rad", default_value="0.0"),
            DeclareLaunchArgument("vision_yaw_rad", default_value="0.0"),
            DeclareLaunchArgument("publish_companion_status", default_value="true"),
            DeclareLaunchArgument("velocity_rate_hz", default_value="20.0"),
            DeclareLaunchArgument("watchdog_timeout_s", default_value="0.5"),
            DeclareLaunchArgument("require_guided_mode", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
