from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    spawn_solver = LaunchConfiguration("spawn_solver")
    spawn_solver_epoch_flag = LaunchConfiguration("spawn_solver_epoch_flag")

    with_epoch = IfCondition(
        PythonExpression(["'", spawn_solver, "' == 'true' and '", spawn_solver_epoch_flag, "' == 'true'"])
    )
    without_epoch = IfCondition(
        PythonExpression(["'", spawn_solver, "' == 'true' and '", spawn_solver_epoch_flag, "' == 'false'"])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_solver", default_value="true"),
            DeclareLaunchArgument("spawn_solver_epoch_flag", default_value="true"),
            ExecuteProcess(
                cmd=["rkk-hand-solver", "--emit-openxr", "--driver-arg=-ef", "--driver-arg=1024"],
                output="screen",
                condition=with_epoch,
            ),
            ExecuteProcess(
                cmd=["rkk-hand-solver", "--emit-openxr"],
                output="screen",
                condition=without_epoch,
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("rkk_hand_bridge"), "launch", "hand_bridge.launch.py"]
                    )
                )
            ),
        ]
    )
