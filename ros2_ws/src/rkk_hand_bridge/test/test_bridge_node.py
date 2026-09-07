import json
import queue
import struct
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage

from rkk_hand_bridge import rgmp_client as rgmp
from rkk_hand_bridge.bridge_node import _LATCHED_QOS, _SENSOR_QOS, BridgeNode
from rkk_hand_bridge.hand_stream import HandStream
from rkk_hand_msgs.msg import HandDescription, HandJoints

_JOINT_NAMES = (
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
)


class FeedableSocket:
    """A `read`/`close`-compatible stand-in for a socket whose bytes the
    test controls the pacing of, so pub/sub discovery can complete
    before a volatile (non-latched) message is published."""

    def __init__(self):
        self._chunks = queue.Queue()
        self._buf = b""

    def feed(self, data: bytes):
        self._chunks.put(data)

    def read(self, n):
        if not self._buf:
            item = self._chunks.get()
            if item is None:
                return b""
            self._buf = item
        chunk, self._buf = self._buf[:n], self._buf[n:]
        return chunk

    def close(self):
        self._chunks.put(None)


def _frame(msg_prefix, payload):
    return struct.pack("<II", msg_prefix, len(payload)) + payload


def _definition_bytes(device_id=7, hand="right"):
    streams = [{"target_frame": f"xr_{name}"} for name in _JOINT_NAMES]
    payload = json.dumps(
        {
            "device_id": device_id,
            "device_type": "solved_hand",
            "timestamp_epoch": "unix_epoch",
            "device_info": {"hand": hand},
            "static_data": [],
            "groups": [{"name": "joints_openxr", "streams": streams}],
        }
    ).encode()
    return _frame(rgmp.MSG_DEFINITION, payload)


def _data_bytes(device_id=7, group_id=0, timestamp_us=1_000_000):
    pose = struct.pack("<7f", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    payload = struct.pack("<IIQ", device_id, group_id, timestamp_us) + pose * rgmp.JOINT_COUNT
    return _frame(rgmp.MSG_DATA, payload)


def _spin_until(executor, condition, timeout_s):
    deadline = time.time() + timeout_s
    while time.time() < deadline and not condition():
        executor.spin_once(timeout_sec=0.1)
    return condition()


def test_publishes_description_and_joints():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        recorder = rclpy.create_node("recorder")

        received = {}
        recorder.create_subscription(
            HandDescription,
            "/rkk_hand_bridge/hand/description",
            lambda msg: received.setdefault("description", msg),
            _LATCHED_QOS,
        )
        joints_sub = recorder.create_subscription(
            HandJoints,
            "/rkk_hand_bridge/hand/joints",
            lambda msg: received.setdefault("joints", msg),
            _SENSOR_QOS,
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: "description" in received, 5.0)
        # wait for the joints subscriber to actually be matched before
        # publishing a volatile (non-latched) message - otherwise it's
        # just gone, correctly, per its QoS.
        assert _spin_until(executor, lambda: joints_sub.get_publisher_count() > 0, 5.0)

        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: "joints" in received, 5.0)

        assert received["description"].hand == HandDescription.HAND_RIGHT
        assert list(received["description"].joint_names) == list(_JOINT_NAMES)
        assert received["joints"].device_id == 7
        assert received["joints"].timestamp_us == 1_000_000
        assert len(received["joints"].joints) == rgmp.JOINT_COUNT
        # epoch mode: 1_000_000 us -> 1s exactly
        assert received["joints"].header.stamp.sec == 1

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_tf_requires_a_parent_frame_id():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("publish_tf", value=True)],
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        sock.feed(_definition_bytes())
        sock.feed(_data_bytes())
        # no parent_frame_id set: should warn and not crash, not broadcast
        deadline = time.time() + 2.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert node._warned_no_parent_frame

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_tf_broadcasts_one_transform_per_joint():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[
                Parameter("publish_tf", value=True),
                Parameter("parent_frame_id", value="world"),
            ],
        )
        recorder = rclpy.create_node("recorder")

        received = []
        tf_sub = recorder.create_subscription(TFMessage, "/tf", received.append, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: tf_sub.get_publisher_count() > 0, 5.0)
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: len(received) > 0, 5.0)

        transforms = received[0].transforms
        assert len(transforms) == rgmp.JOINT_COUNT
        assert transforms[0].header.frame_id == "world"
        assert transforms[1].child_frame_id == "rkk_right_hand_xr_wrist"

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()
