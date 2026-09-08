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

MODES = ("external", "solver", "launch")


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
            if section not in config:
                raise RuntimeError(f"unknown section in {path}: {section}")
            config[section].update(values or {})
    return config


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = _load(arg("config"))
    driver, solver = config["driver"], config["solver"]

    # Explicit launch argument wins over the config file.
    mode = arg("driver_mode") or driver["mode"]
    if mode not in MODES:
        raise RuntimeError(f"driver.mode must be one of {', '.join(MODES)}, got: {mode}")

    actions = []
    # driver_args only reach a driver the solver itself starts; in the other
    # modes the driver's arguments come from driver.args or from the user.
    driver_args = list(solver.get("driver_args") or []) if mode == "solver" else []

    if mode == "launch":
        actions.append(
            ExecuteProcess(cmd=[driver["executable"], *driver["args"]], output="screen")
        )

    if arg("spawn_solver").strip().lower() in ("1", "true", "yes"):
        cmd = [solver["executable"], *solver["args"]]
        if mode == "solver":
            cmd += [f"--driver-arg={value}" for value in driver_args]
        else:
            # Either we started the driver, or the user did. Don't start a second.
            cmd.append("--no-driver")
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
            # Empty means "use driver.mode from the config file".
            DeclareLaunchArgument("driver_mode", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
