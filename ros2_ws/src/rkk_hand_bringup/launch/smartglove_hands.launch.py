import os

import yaml
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

# Used when a key is absent from the config file, so a partial config
# (e.g. only `driver:`) still produces the documented command lines.
DEFAULTS = {
    "driver": {
        "spawn": False,
        "executable": "rokoko-sdk",
        "args": ["-vv", "--auto-stream-usb", "-ef", "1024"],
    },
    "solver": {
        "executable": "rkk-hand-solver",
        "args": ["--emit-openxr"],
        "driver_args": ["-ef", "1024"],
    },
}


def _load(path):
    """Merge the config file over DEFAULTS, one level deep."""
    merged = {section: dict(values) for section, values in DEFAULTS.items()}
    if not path:
        return merged
    if not os.path.exists(path):
        raise RuntimeError(f"upstream config not found: {path}")
    with open(path) as handle:
        loaded = yaml.safe_load(handle) or {}
    for section, values in loaded.items():
        if section not in merged:
            raise RuntimeError(f"unknown section in {path}: {section}")
        merged[section].update(values or {})
    return merged


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes")


def _setup(context, *_args, **_kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config = _load(arg("config"))
    driver, solver = config["driver"], config["solver"]

    # Precedence: explicit launch argument > config file > DEFAULTS.
    spawn_solver = _as_bool(arg("spawn_solver"))
    spawn_driver = _as_bool(arg("spawn_driver")) if arg("spawn_driver") else _as_bool(driver["spawn"])

    driver_args = list(driver["args"])
    solver_driver_args = list(solver["driver_args"])
    # Back-compat: the retired epoch toggle still drops the -ef pair.
    if arg("spawn_solver_epoch_flag") and not _as_bool(arg("spawn_solver_epoch_flag")):
        solver_driver_args = []
        driver_args = [a for a in driver_args if a not in ("-ef", "1024")]

    actions = []

    if spawn_driver:
        actions.append(
            ExecuteProcess(cmd=[driver["executable"], *driver_args], output="screen")
        )

    if spawn_solver:
        cmd = [solver["executable"], *solver["args"]]
        if spawn_driver:
            # The driver is already ours; don't let the solver start a second.
            cmd.append("--no-driver")
        else:
            for driver_arg in solver_driver_args:
                cmd.append(f"--driver-arg={driver_arg}")
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
    default_config = PathJoinSubstitution(
        [FindPackageShare("rkk_hand_bringup"), "config", "upstream.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("spawn_solver", default_value="true"),
            # Empty means "defer to the config file".
            DeclareLaunchArgument("spawn_driver", default_value=""),
            DeclareLaunchArgument("spawn_solver_epoch_flag", default_value=""),
            OpaqueFunction(function=_setup),
        ]
    )
