from catkin_pkg.package import parse_package
from setuptools import find_packages, setup

package_name = "rkk_hand_bridge"

setup(
    name=package_name,
    version=parse_package("package.xml").version,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/hand_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Rokoko",
    maintainer_email="anastasia.panaretou@rokoko.com",
    description="ROS2 bridge for rkk-hand-solver's solved SmartGlove hand output.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bridge_node = rkk_hand_bridge.bridge_node:main",
        ],
    },
)
