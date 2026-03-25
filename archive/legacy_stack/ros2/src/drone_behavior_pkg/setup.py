from setuptools import find_packages, setup

import glob
import os

package_name = "drone_behavior_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob.glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "config", "scenarios"),
            glob.glob(os.path.join("config", "scenarios", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sezgin_atabas",
    maintainer_email="atabassezgin@gmail.com",
    description="Behavior orchestration package for multi-target engagement.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "engagement_manager_node = drone_behavior_pkg.engagement_manager_node:main",
            "health_monitor_node = drone_behavior_pkg.health_monitor_node:main",
        ],
    },
)
