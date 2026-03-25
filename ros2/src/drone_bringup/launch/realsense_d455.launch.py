from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _load_profile(path: str) -> dict[str, object]:
    profile_path = Path(path)
    if not profile_path.exists():
        raise FileNotFoundError(f"RealSense profile not found: {profile_path}")
    data = yaml.safe_load(profile_path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping in {profile_path}")
    return data


def _to_launch_value(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def launch_setup(context, *args, **kwargs):
    del args, kwargs

    bringup_share = get_package_share_directory("drone_bringup")
    default_profile = Path(bringup_share) / "config" / "realsense_d455_vio.yaml"
    config_file = LaunchConfiguration("config_file").perform(context).strip()
    profile_path = config_file or str(default_profile)
    profile = _load_profile(profile_path)

    camera_namespace = LaunchConfiguration("camera_namespace").perform(context).strip()
    camera_name = LaunchConfiguration("camera_name").perform(context).strip()
    serial_no = LaunchConfiguration("serial_no").perform(context).strip()
    log_level = LaunchConfiguration("log_level").perform(context).strip() or "info"

    if camera_namespace:
        profile["camera_namespace"] = camera_namespace
    if camera_name:
        profile["camera_name"] = camera_name
    if serial_no:
        profile["serial_no"] = serial_no

    rs_launch = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    launch_arguments = {
        key: _to_launch_value(value)
        for key, value in profile.items()
    }
    launch_arguments["log_level"] = log_level

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory("drone_bringup")
    default_profile = str(Path(bringup_share) / "config" / "realsense_d455_vio.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_profile),
            DeclareLaunchArgument("camera_namespace", default_value=""),
            DeclareLaunchArgument("camera_name", default_value=""),
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            OpaqueFunction(function=launch_setup),
        ]
    )
