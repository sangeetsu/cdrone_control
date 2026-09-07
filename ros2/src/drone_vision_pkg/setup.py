from setuptools import find_packages, setup

import glob
import os

package_name = "drone_vision_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob.glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sezgin_atabas",
    maintainer_email="atabassezgin@gmail.com",
    description="Tracking-first perception nodes for cdrone_control",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "realsense_tracker_node = drone_vision_pkg.realsense_tracker_node:main",
            "target_map_node = drone_vision_pkg.target_map_node:main",
            "tracking_metrics_node = drone_vision_pkg.tracking_metrics:main",
            "tracking_metrics_postprocess = "
            "drone_vision_pkg.tracking_metrics:postprocess_main",
            "milestone_tracking_compare_report = "
            "drone_vision_pkg.tracking_metrics:compare_main",
            "world_track_compare_report = "
            "drone_vision_pkg.world_track_compare_report:main",
            "zed_depth_ladder_report = drone_vision_pkg.zed_depth_ladder:main",
        ],
    },
)
