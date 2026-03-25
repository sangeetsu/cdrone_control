from setuptools import find_packages, setup

package_name = "drone_monitor_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sezgin_atabas",
    maintainer_email="atabassezgin@gmail.com",
    description="Telemetry aggregation and terminal dashboard for cdrone_control.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "telemetry_aggregator_node = drone_monitor_pkg.telemetry_aggregator_node:main",
            "dashboard_tui_node = drone_monitor_pkg.dashboard_tui_node:main",
        ],
    },
)
