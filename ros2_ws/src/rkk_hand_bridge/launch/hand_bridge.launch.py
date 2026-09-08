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
        DeclareLaunchArgument("publish_markers", default_value="false"),
        DeclareLaunchArgument("marker_lifetime_s", default_value="0.5"),
        DeclareLaunchArgument("marker_joint_scale", default_value="1.0"),
        DeclareLaunchArgument("marker_max_joint_radius_m", default_value="0.010"),
        DeclareLaunchArgument("marker_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("parent_frame_id", default_value=""),
        DeclareLaunchArgument("calibrate_facing_rad", default_value="0.0"),
        DeclareLaunchArgument("reconnect_delay_s", default_value="0.5"),
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
                "publish_markers": LaunchConfiguration("publish_markers"),
                "marker_lifetime_s": LaunchConfiguration("marker_lifetime_s"),
                "marker_joint_scale": LaunchConfiguration("marker_joint_scale"),
                "marker_max_joint_radius_m": LaunchConfiguration("marker_max_joint_radius_m"),
                "marker_rate_hz": LaunchConfiguration("marker_rate_hz"),
                "parent_frame_id": LaunchConfiguration("parent_frame_id"),
                "calibrate_facing_rad": LaunchConfiguration("calibrate_facing_rad"),
                "reconnect_delay_s": LaunchConfiguration("reconnect_delay_s"),
            }
        ],
    )

    return LaunchDescription(args + [node])
