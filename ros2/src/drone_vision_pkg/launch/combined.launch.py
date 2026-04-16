from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("log_level", default_value="info"),
            LogInfo(
                msg=(
                    "drone_vision_pkg combined scaffolding is reserved for a "
                    "later milestone."
                )
            ),
        ]
    )
