"""rclpy node: republishes rkk-hand-solver's solved-hand stream as ROS2
topics. Thin glue over hand_stream, frame_convert, and timestamps -
those hold the actual logic."""

import threading

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rkk_hand_msgs.msg import HandDescription, HandJoints
from tf2_ros import TransformBroadcaster

from rkk_hand_bridge import frame_convert as fc
from rkk_hand_bridge.hand_stream import HandStream
from rkk_hand_bridge.timestamps import StampSource

_HAND_CODE = {"left": HandDescription.HAND_LEFT, "right": HandDescription.HAND_RIGHT}

_LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)
_SENSOR_QOS = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def _hand_code(hand: str) -> int:
    return _HAND_CODE.get(hand, HandDescription.HAND_LEFT)


def _stamp_from_ns(stamp_ns: int) -> TimeMsg:
    return TimeMsg(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def _to_pose(position, orientation) -> Pose:
    return Pose(
        position=Point(x=position[0], y=position[1], z=position[2]),
        orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]),
    )


def _joint_frame_id(hand: str, joint_name: str) -> str:
    return f"rkk_{hand}_hand_xr_{joint_name}"


class BridgeNode(Node):
    def __init__(self, stream: HandStream | None = None, **node_kwargs):
        super().__init__("rkk_hand_bridge", **node_kwargs)

        self.declare_parameter("solver_host", "127.0.0.1")
        self.declare_parameter("solver_port", 12277)
        self.declare_parameter("reconnect_backoff_s", 0.5)
        self.declare_parameter("stamp_source", "auto")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("parent_frame_id", "")

        self._description_pub = self.create_publisher(HandDescription, "~/hand/description", _LATCHED_QOS)
        self._joints_pub = self.create_publisher(HandJoints, "~/hand/joints", _SENSOR_QOS)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._warned_no_parent_frame = False

        self._published_description = set()
        self._rebases = {}
        self._stamp_sources = {}
        self._last_timestamp_us = {}

        self._stream = stream or HandStream(
            self.get_parameter("solver_host").value,
            self.get_parameter("solver_port").value,
            reconnect_backoff_s=self.get_parameter("reconnect_backoff_s").value,
            on_unusable_hand=self._on_unusable_hand,
        )
        self._stream.start()

        self._stopping = threading.Event()
        self._watcher = threading.Thread(target=self._watch, daemon=True)
        self._watcher.start()

    def destroy_node(self):
        self._stopping.set()
        self._stream.stop()
        self._watcher.join(timeout=5.0)
        super().destroy_node()

    def _on_unusable_hand(self, device_id: int, reason: str):
        self.get_logger().warning(f"solved hand {device_id}: {reason}")

    def _watch(self):
        seen = 0
        while not self._stopping.is_set():
            try:
                seen = self._stream.wait(seen, timeout=1.0)
            except TimeoutError:
                continue
            self._publish_new_state()

    def _publish_new_state(self):
        hands = self._stream.hands()
        frames = self._stream.latest()

        for device_id in list(self._published_description):
            if device_id not in hands:
                self._published_description.discard(device_id)
                self._rebases.pop(device_id, None)
                self._stamp_sources.pop(device_id, None)
                self._last_timestamp_us.pop(device_id, None)

        for device_id, definition in hands.items():
            if device_id not in self._published_description:
                self._publish_description(definition)
                self._published_description.add(device_id)

        for device_id, frame in frames.items():
            definition = hands.get(device_id)
            if definition is None:
                continue
            if self._last_timestamp_us.get(device_id) == frame.timestamp_us:
                continue
            self._last_timestamp_us[device_id] = frame.timestamp_us
            self._publish_joints(definition, frame)

    def _publish_description(self, definition):
        msg = HandDescription()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.hand = _hand_code(definition.hand)
        msg.device_id = definition.device_id
        msg.timestamp_epoch = definition.timestamp_epoch or ""
        msg.joint_names = list(definition.joint_names)
        msg.joint_radii = list(definition.joint_radii)
        self._description_pub.publish(msg)

    def _publish_joints(self, definition, frame):
        rebase = self._rebases.setdefault(frame.device_id, fc.Rebase())
        stamp_source = self._stamp_sources.setdefault(
            frame.device_id, StampSource(self.get_parameter("stamp_source").value)
        )

        receive_time_ns = self.get_clock().now().nanoseconds
        stamp_ns = stamp_source.compute(frame.timestamp_us, definition.timestamp_epoch, receive_time_ns)
        if stamp_ns is None:
            return  # backwards jump - drop, don't publish a stale-looking stamp

        stamp = _stamp_from_ns(stamp_ns)
        rebased = [rebase.apply(j.position, j.orientation) for j in frame.joints]

        msg = HandJoints()
        msg.header.stamp = stamp
        msg.hand = _hand_code(definition.hand)
        msg.device_id = frame.device_id
        msg.timestamp_us = frame.timestamp_us
        msg.joints = [_to_pose(position, orientation) for position, orientation in rebased]
        self._joints_pub.publish(msg)

        if self.get_parameter("publish_tf").value:
            self._broadcast_tf(definition, rebased, stamp)

    def _broadcast_tf(self, definition, rebased, stamp: TimeMsg):
        parent_frame_id = self.get_parameter("parent_frame_id").value
        if not parent_frame_id:
            if not self._warned_no_parent_frame:
                self.get_logger().warning("publish_tf is set but parent_frame_id is empty; not broadcasting")
                self._warned_no_parent_frame = True
            return

        transforms = []
        for name, (position, orientation) in zip(definition.joint_names, rebased):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent_frame_id
            t.child_frame_id = _joint_frame_id(definition.hand, name)
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = position
            t.transform.rotation = Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
            transforms.append(t)
        self._tf_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
