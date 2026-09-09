from catkin_pkg.package import parse_package
from setuptools import find_packages, setup

package_name = "rkk_hand_bringup"

setup(
    name=package_name,
    version=parse_package("package.xml").version,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/smartglove_hands.launch.py"]),
        ("share/" + package_name + "/config", ["config/upstream.yaml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Rokoko",
    maintainer_email="anastasia.panaretou@rokoko.com",
    description="Launch files bringing up rkk-hand-solver and rkk_hand_bridge together.",
    license="Apache-2.0",
)
