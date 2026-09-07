from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("solver_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("solver_port", default_value="12277"),
        DeclareLaunchArgument("stamp_source", default_value="auto"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        DeclareLaunchArgument("parent_frame_id", default_value=""),
        DeclareLaunchArgument("calibrate_facing_rad", default_value="0.0"),
        DeclareLaunchArgument("reconnect_backoff_s", default_value="0.5"),
    ]

    node = Node(
        package="rkk_hand_bridge",
        executable="bridge_node",
        name="rkk_hand_bridge",
        output="screen",
        parameters=[
            {
                "solver_host": LaunchConfiguration("solver_host"),
                "solver_port": LaunchConfiguration("solver_port"),
                "stamp_source": LaunchConfiguration("stamp_source"),
                "publish_tf": LaunchConfiguration("publish_tf"),
                "parent_frame_id": LaunchConfiguration("parent_frame_id"),
                "calibrate_facing_rad": LaunchConfiguration("calibrate_facing_rad"),
                "reconnect_backoff_s": LaunchConfiguration("reconnect_backoff_s"),
            }
        ],
    )

    return LaunchDescription(args + [node])
