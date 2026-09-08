from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rkk_hand_bridge import constants

PARAMETERS = {
    "solver_host": constants.SOLVER_HOST,
    "solver_port": constants.SOLVER_PORT,
    "reconnect_delay_s": constants.RECONNECT_DELAY_S,
    "stamp_source": constants.STAMP_SOURCE,
    "parent_frame_id": constants.PARENT_FRAME_ID,
    "publish_tf": constants.PUBLISH_TF,
    "publish_markers": constants.PUBLISH_MARKERS,
    "marker_rate_hz": constants.MARKER_RATE_HZ,
    "marker_lifetime_s": constants.MARKER_LIFETIME_S,
    "marker_joint_scale": constants.MARKER_JOINT_SCALE,
    "marker_max_joint_radius_m": constants.MARKER_MAX_JOINT_RADIUS_M,
    "marker_hand_spacing_m": constants.MARKER_HAND_SPACING_M,
    "calibrate_facing_rad": constants.CALIBRATE_FACING_RAD,
    "calibrate_max_disagreement_rad": constants.CALIBRATE_MAX_DISAGREEMENT_RAD,
}


def _as_launch_default(value) -> str:
    return str(value).lower() if isinstance(value, bool) else str(value)


def generate_launch_description():
    args = [
        DeclareLaunchArgument(name, default_value=_as_launch_default(value))
        for name, value in PARAMETERS.items()
    ]
    node = Node(
        package="rkk_hand_bridge",
        executable="bridge_node",
        name="rkk_hand_bridge",
        output="screen",
        parameters=[{name: LaunchConfiguration(name) for name in PARAMETERS}],
    )
    return LaunchDescription(args + [node])
