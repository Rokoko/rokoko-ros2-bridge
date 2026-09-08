import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

SECTIONS = ("solver", "driver")


def _shipped_config():
    return os.path.join(
        get_package_share_directory("rkk_hand_bringup"), "config", "upstream.yaml"
    )


def _read(path):
    if not os.path.exists(path):
        raise RuntimeError(f"upstream config not found: {path}")
    with open(path) as handle:
        return yaml.safe_load(handle) or {}


def _load(path):
    """The shipped config is the default; `path` merges over it, one level deep."""
    config = _read(_shipped_config())
    if path and os.path.abspath(path) != os.path.abspath(_shipped_config()):
        for section, values in _read(path).items():
            if section not in SECTIONS:
                raise RuntimeError(f"unknown section in {path}: {section}")
            config.setdefault(section, {}).update(values or {})
    return config


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = _load(arg("config"))
    solver = config["solver"]
    # Optional. Present means this launch file starts the driver itself.
    driver = config.get("driver") or {}

    solver_args = list(solver.get("args") or [])
    if driver and "--no-driver" not in solver_args:
        raise RuntimeError(
            "config has a `driver` section, so this launch file starts the driver, "
            "but solver.args does not contain --no-driver — the solver would start "
            "a second one. Add --no-driver to solver.args, or drop the driver section."
        )

    actions = []
    if driver:
        actions.append(
            ExecuteProcess(
                cmd=[driver["executable"], *(driver.get("args") or [])], output="screen"
            )
        )

    if arg("spawn_solver").strip().lower() in ("1", "true", "yes"):
        # solver.args is passed through verbatim; whether the solver spawns a
        # driver is decided by what is in it (i.e. --no-driver), not by us.
        cmd = [solver["executable"], *solver_args]
        cmd += [f"--driver-arg={value}" for value in solver.get("driver_args") or []]
        actions.append(ExecuteProcess(cmd=cmd, output="screen"))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("rkk_hand_bridge"), "launch", "hand_bridge.launch.py"]
                )
            )
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("spawn_solver", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
