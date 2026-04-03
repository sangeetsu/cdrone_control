import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    log_level = LaunchConfiguration("log_level")
    mavros_namespace = LaunchConfiguration("mavros_namespace")
    drone_id = LaunchConfiguration("drone_id")
    pose_source = LaunchConfiguration("pose_source").perform(context).strip().lower()
    source_pose_topic = LaunchConfiguration("source_pose_topic").perform(context).strip()
    rigid_body_name = LaunchConfiguration("rigid_body_name").perform(context).strip()
    default_optitrack_topic = f"/vrpn_mocap/{rigid_body_name}/pose"

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

    if pose_source == "optitrack":
        vrpn_share = get_package_share_directory("vrpn_mocap")
        nodes.append(
            Node(
                package="vrpn_mocap",
                executable="client_node",
                namespace="vrpn_mocap",
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
            output="screen",
            parameters=[
                {
                    "drone_id": drone_id,
                    "source_pose_topic": adapter_source_topic,
                    "output_pose_topic": LaunchConfiguration("external_pose_topic"),
                    "map_frame": LaunchConfiguration("map_frame"),
                    "position_offset_m": LaunchConfiguration("position_offset_m"),
                    "rpy_offset_rad": LaunchConfiguration("rpy_offset_rad"),
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

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument("drone_id", default_value="drone01"),
            DeclareLaunchArgument("mavros_namespace", default_value="mavros"),
            DeclareLaunchArgument("pose_source", default_value="optitrack"),
            DeclareLaunchArgument("source_pose_topic", default_value=""),
            DeclareLaunchArgument("external_pose_topic", default_value=""),
            DeclareLaunchArgument("bridge_input_pose_topic", default_value=""),
            DeclareLaunchArgument("legacy_input_pose_topic", default_value=""),
            DeclareLaunchArgument("output_pose_topic", default_value=""),
            DeclareLaunchArgument("publish_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("input_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("adapter_timeout_s", default_value="0.25"),
            DeclareLaunchArgument("publish_companion_status", default_value="true"),
            DeclareLaunchArgument("restamp_with_local_clock", default_value="false"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument(
                "position_offset_m", default_value="[0.0, 0.0, 0.0]"
            ),
            DeclareLaunchArgument(
                "rpy_offset_rad", default_value="[0.0, 0.0, 0.0]"
            ),
            DeclareLaunchArgument("optitrack_server", default_value="localhost"),
            DeclareLaunchArgument("optitrack_port", default_value="3883"),
            DeclareLaunchArgument("rigid_body_name", default_value="rigidbody3"),
            DeclareLaunchArgument("use_vrpn_timestamps", default_value="false"),
            OpaqueFunction(function=launch_setup),
        ]
    )
