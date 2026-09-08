import json
import queue
import struct
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from rkk_hand_bridge import rgmp
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
    """A `recv`/`close`-compatible stand-in for a socket (matches the
    real socket API) whose bytes the test controls the pacing of, so
    pub/sub discovery can complete before a volatile (non-latched)
    message is published."""

    def __init__(self):
        self._chunks = queue.Queue()
        self._buf = b""

    def feed(self, data: bytes):
        self._chunks.put(data)

    def recv(self, n):
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


def _disconnect_bytes(device_id=7):
    return _frame(rgmp.MSG_DISCONNECT, struct.pack("<I", device_id))


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


def _call_calibrate(executor, recorder, node):
    client = recorder.create_client(Trigger, "/rkk_hand_bridge/calibrate")
    assert _spin_until(executor, lambda: client.wait_for_service(timeout_sec=0.0), 5.0)
    future = client.call_async(Trigger.Request())
    assert _spin_until(executor, future.done, 5.0)
    client.destroy()
    return future.result()


def test_calibrate_fails_with_no_hand_data_yet():
    rclpy.init()
    try:
        stream = HandStream("h", 0, connect=lambda: FeedableSocket())
        node = BridgeNode(stream=stream)
        recorder = rclpy.create_node("recorder")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        result = _call_calibrate(executor, recorder, node)
        assert result.success is False

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_calibrate_applies_offset_to_later_frames():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        recorder = rclpy.create_node("recorder")

        received = []
        joints_sub = recorder.create_subscription(
            HandJoints, "/rkk_hand_bridge/hand/joints", received.append, _SENSOR_QOS
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: joints_sub.get_publisher_count() > 0, 5.0)
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: len(received) > 0, 5.0)

        result = _call_calibrate(executor, recorder, node)
        assert result.success is True
        assert node._yaw_offset != 0.0

        received.clear()
        sock.feed(_data_bytes(timestamp_us=2_000_000))
        assert _spin_until(executor, lambda: len(received) > 0, 5.0)

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_diagnostics_reflects_connection_and_calibration_state():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        recorder = rclpy.create_node("recorder")

        statuses = []
        recorder.create_subscription(DiagnosticArray, "/diagnostics", statuses.append, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: statuses and statuses[-1].status, 5.0)
        assert statuses[-1].status[0].level == DiagnosticStatus.WARN

        statuses.clear()
        result = _call_calibrate(executor, recorder, node)
        assert result.success is True
        assert _spin_until(executor, lambda: statuses, 5.0)
        assert statuses[-1].status[0].level == DiagnosticStatus.OK

        statuses.clear()
        sock.feed(_disconnect_bytes())
        assert _spin_until(executor, lambda: statuses, 5.0)
        assert statuses[-1].status[0].level == DiagnosticStatus.ERROR
        assert statuses[-1].status[0].message == "disconnected"

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_markers_requires_a_parent_frame_id():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("publish_markers", value=True)],
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        sock.feed(_definition_bytes())
        sock.feed(_data_bytes())
        # no parent_frame_id set: should warn and not crash, not publish
        deadline = time.time() + 2.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert node._warned_no_marker_frame

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_publish_markers_draws_the_hand_in_the_parent_frame():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[
                Parameter("publish_markers", value=True),
                Parameter("parent_frame_id", value="world"),
            ],
        )
        recorder = rclpy.create_node("marker_recorder")
        received = []
        sub = recorder.create_subscription(
            MarkerArray, "/rkk_hand_bridge/hand/markers", received.append, 10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: sub.get_publisher_count() > 0, 5.0)
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: received, 5.0)

        markers = received[0].markers
        spheres = [m for m in markers if m.type == Marker.SPHERE]
        bones = [m for m in markers if m.type == Marker.LINE_LIST]
        assert len(spheres) == rgmp.JOINT_COUNT
        assert len(bones) == 1
        assert all(m.header.frame_id == "world" for m in markers)
        # markers and TF/HandJoints must describe the same hand, so the
        # marker stamp is the joints stamp, not a fresh clock reading.
        assert all(m.header.stamp.sec == 1 for m in markers)
        assert all("right" in m.ns for m in markers)

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_markers_are_not_published_unless_asked_for():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("parent_frame_id", value="world")],
        )
        recorder = rclpy.create_node("marker_recorder")
        received = []
        recorder.create_subscription(
            MarkerArray, "/rkk_hand_bridge/hand/markers", received.append, 10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        sock.feed(_data_bytes())
        deadline = time.time() + 2.0
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert received == []

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_disconnecting_hand_clears_its_markers():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[
                Parameter("publish_markers", value=True),
                Parameter("parent_frame_id", value="world"),
            ],
        )
        recorder = rclpy.create_node("marker_recorder")
        received = []
        sub = recorder.create_subscription(
            MarkerArray, "/rkk_hand_bridge/hand/markers", received.append, 10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: sub.get_publisher_count() > 0, 5.0)
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: received, 5.0)

        sock.feed(_disconnect_bytes())
        assert _spin_until(
            executor,
            lambda: any(
                m.action == Marker.DELETEALL for array in received for m in array.markers
            ),
            5.0,
        )

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_markers_are_throttled_below_the_frame_rate():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("marker_rate_hz", value=30.0)],
        )
        # Two frames back to back arrive far faster than 1/30s apart, so
        # only the first is due.
        assert node._due_for_markers(7) is True
        assert node._due_for_markers(7) is False
        # a second hand is throttled independently
        assert node._due_for_markers(8) is True

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_zero_marker_rate_publishes_every_frame():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("marker_rate_hz", value=0.0)],
        )
        assert all(node._due_for_markers(7) for _ in range(5))

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_changing_stamp_source_takes_effect_on_a_connected_hand():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        sock.feed(_definition_bytes())
        sock.feed(_data_bytes())
        assert _spin_until(executor, lambda: 7 in node._stamp_sources, 5.0)
        assert node._stamp_sources[7].mode == "auto"

        node.set_parameters([Parameter("stamp_source", value="receive")])
        sock.feed(_data_bytes(timestamp_us=2_000_000))
        assert _spin_until(
            executor, lambda: node._stamp_sources[7].mode == "receive", 5.0
        )

        node.destroy_node()
    finally:
        rclpy.shutdown()


def _definition_without_openxr(device_id=7):
    payload = json.dumps(
        {
            "device_id": device_id,
            "device_type": "solved_hand",
            "timestamp_epoch": "unix_epoch",
            "device_info": {"hand": "right"},
            "static_data": [],
            "groups": [{"name": "joints_local", "streams": []}],
        }
    ).encode()
    return _frame(rgmp.MSG_DEFINITION, payload)


def test_an_unsupported_hand_is_reported_on_diagnostics():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        recorder = rclpy.create_node("diag_recorder")
        received = []
        recorder.create_subscription(DiagnosticArray, "/diagnostics", received.append, 10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_without_openxr())
        assert _spin_until(
            executor,
            lambda: any(
                s.level == DiagnosticStatus.ERROR and "unsupported" in s.name
                for a in received
                for s in a.status
            ),
            5.0,
        )
        status = next(
            s for a in received for s in a.status if "unsupported" in s.name
        )
        assert status.hardware_id == "7"
        assert "--emit-openxr" in status.message
        assert status.values[0].value == rgmp.MISSING_OPENXR_GROUP

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_a_hand_that_becomes_supported_clears_its_error():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(stream=stream)
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        sock.feed(_definition_without_openxr())
        assert _spin_until(executor, lambda: 7 in node._unsupported, 5.0)

        sock.feed(_definition_bytes())
        assert _spin_until(executor, lambda: 7 not in node._unsupported, 5.0)

        node.destroy_node()
    finally:
        rclpy.shutdown()
