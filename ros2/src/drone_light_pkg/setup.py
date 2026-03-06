from setuptools import find_packages, setup

package_name = "drone_light_pkg"

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
    description="GPIO/PWM based spotlight control node.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "light_controller_node = drone_light_pkg.light_controller_node:main",
        ],
    },
)
