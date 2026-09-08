import json
import math
import queue
import struct
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from rkk_hand_bridge import frame_convert as fc
from rkk_hand_bridge import rgmp
from rkk_hand_bridge.rgmp import decode_definition as _raw_decode
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


def _decode_definition(frame_bytes):
    return _raw_decode(frame_bytes[8:])


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
        assert node._due_for_markers(7, 1) is True
        assert node._due_for_markers(7, 1) is False
        assert node._due_for_markers(8, 1) is True

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
        assert all(node._due_for_markers(7, 1) for _ in range(5))

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_hands_share_one_marker_budget():
    rclpy.init()
    try:
        sock = FeedableSocket()
        stream = HandStream("h", 0, connect=lambda: sock)
        node = BridgeNode(
            stream=stream,
            parameter_overrides=[Parameter("marker_rate_hz", value=30.0)],
        )
        clock_ns = [0]
        node.get_clock().now = lambda: SimpleNamespace(nanoseconds=clock_ns[0])

        assert node._due_for_markers(7, 2) is True
        assert node._due_for_markers(8, 2) is True

        # 1/30s in: with one hand each would be due again, but the budget
        # is shared, so each hand only gets half of it.
        clock_ns[0] = int(1e9 / 30) + 1
        assert node._due_for_markers(7, 2) is False
        assert node._due_for_markers(8, 2) is False

        clock_ns[0] = int(2 * 1e9 / 30) + 1
        assert node._due_for_markers(7, 2) is True
        assert node._due_for_markers(8, 2) is True

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


class _OneUpdateStream:
    """Wakes _watch exactly once with a hand to publish, then goes idle."""

    def __init__(self, definition=None):
        self.woken = False
        self._definition = definition

    def wait(self, seen, timeout=None):
        if self.woken:
            return seen
        self.woken = True
        return seen + 1

    def hands(self):
        return {7: self._definition} if self._definition else {}

    def latest(self):
        return {}

    def stop(self):
        pass


def test_watch_exits_quietly_when_the_context_dies_mid_publish():
    rclpy.init()
    sock = FeedableSocket()
    node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
    # rclpy.shutdown() tears the context down and destroys the publishers;
    # SIGINT does the same underneath a watch thread already in flight.
    definition = _decode_definition(_definition_bytes())
    rclpy.shutdown()
    assert not node.context.ok()
    with pytest.raises(Exception):
        node._publish_description(definition)

    node._stream = _OneUpdateStream(definition)
    node._stopping.clear()
    node._watch()  # must return rather than raise out of the thread


def test_watch_still_raises_when_the_context_is_healthy():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
        node._stream = _OneUpdateStream(_decode_definition(_definition_bytes()))

        def boom():
            raise ValueError("a real failure, not a shutdown")

        node._publish_new_state = boom
        with pytest.raises(ValueError):
            node._watch()

        node.destroy_node()
    finally:
        rclpy.shutdown()


def _data_bytes_facing(device_id, yaw_deg, group_id=0, timestamp_us=1_000_000):
    """Frame whose wrist is yawed `yaw_deg` about the xr_base up axis."""
    identity = struct.pack("<7f", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    qx, qy, qz, qw = fc.axis_angle((0.0, 1.0, 0.0), math.radians(yaw_deg))
    wrist = struct.pack("<7f", 0.0, 0.0, 0.0, qx, qy, qz, qw)
    wrist_index = _JOINT_NAMES.index("wrist")
    poses = b"".join(
        wrist if i == wrist_index else identity for i in range(rgmp.JOINT_COUNT)
    )
    payload = struct.pack("<IIQ", device_id, group_id, timestamp_us) + poses
    return _frame(rgmp.MSG_DATA, payload)


def _two_hands(sock, executor, node, left_yaw_deg, right_yaw_deg):
    sock.feed(_definition_bytes(device_id=7, hand="left"))
    sock.feed(_definition_bytes(device_id=8, hand="right"))
    sock.feed(_data_bytes_facing(7, left_yaw_deg))
    sock.feed(_data_bytes_facing(8, right_yaw_deg))
    assert _spin_until(executor, lambda: len(node._stream.latest()) == 2, 5.0)


def test_calibrate_names_the_hand_it_sampled():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
        recorder = rclpy.create_node("recorder")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        _two_hands(sock, executor, node, 0.0, 5.0)
        result = _call_calibrate(executor, recorder, node)
        assert result.success is True
        assert "left hand" in result.message
        assert "device 7" in result.message

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_calibrate_refuses_hands_that_disagree():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
        recorder = rclpy.create_node("recorder")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        _two_hands(sock, executor, node, 0.0, 45.0)
        result = _call_calibrate(executor, recorder, node)
        assert result.success is False
        assert "45 degrees" in result.message
        assert node._yaw_offset == 0.0
        assert node._ever_calibrated is False

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_calibrate_accepts_a_wide_disagreement_when_allowed():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(
            stream=HandStream("h", 0, connect=lambda: sock),
            parameter_overrides=[
                Parameter("calibrate_max_disagreement_rad", value=math.pi),
            ],
        )
        recorder = rclpy.create_node("recorder")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        _two_hands(sock, executor, node, 0.0, 45.0)
        result = _call_calibrate(executor, recorder, node)
        assert result.success is True

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_calibrate_is_deterministic_across_two_hands():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
        recorder = rclpy.create_node("recorder")
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        _two_hands(sock, executor, node, 0.0, 5.0)
        first = _call_calibrate(executor, recorder, node)
        # the newest frame is now the right hand's, but the lowest device
        # id still decides, so the answer does not move.
        sock.feed(_data_bytes_facing(8, 5.0, timestamp_us=9_000_000))
        assert _spin_until(
            executor, lambda: node._stream.latest()[8].timestamp_us == 9_000_000, 5.0
        )
        second = _call_calibrate(executor, recorder, node)
        assert first.message == second.message

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_hands_are_drawn_apart_but_published_together():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(
            stream=HandStream("h", 0, connect=lambda: sock),
            parameter_overrides=[
                Parameter("publish_markers", value=True),
                Parameter("parent_frame_id", value="world"),
                Parameter("marker_hand_spacing_m", value=0.4),
            ],
        )
        recorder = rclpy.create_node("recorder")
        markers, joints = [], []
        sub = recorder.create_subscription(
            MarkerArray, "/rkk_hand_bridge/hand/markers", markers.append, 10
        )
        recorder.create_subscription(
            HandJoints, "/rkk_hand_bridge/hand/joints", joints.append, _SENSOR_QOS
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes(device_id=7, hand="left"))
        sock.feed(_definition_bytes(device_id=8, hand="right"))
        assert _spin_until(executor, lambda: sub.get_publisher_count() > 0, 5.0)
        sock.feed(_data_bytes(device_id=7))
        sock.feed(_data_bytes(device_id=8))
        assert _spin_until(
            executor,
            lambda: {m.ns.split("_")[1] for a in markers for m in a.markers} == {"left", "right"},
            5.0,
        )

        drawn = {}
        for array in markers:
            for marker in array.markers:
                if marker.type == Marker.SPHERE and marker.id == 0:
                    drawn[marker.ns] = marker.pose.position.y
        assert drawn["rkk_left_hand/joints"] == -0.2
        assert drawn["rkk_right_hand/joints"] == 0.2

        # the data carries no such offset: every joint is still at the origin
        assert joints
        assert all(j.joints[0].position.y == 0.0 for j in joints)

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_an_unlabelled_hand_is_not_published_as_left():
    rclpy.init()
    try:
        sock = FeedableSocket()
        node = BridgeNode(stream=HandStream("h", 0, connect=lambda: sock))
        recorder = rclpy.create_node("recorder")
        received = []
        recorder.create_subscription(
            HandDescription, "/rkk_hand_bridge/hand/description",
            received.append, _LATCHED_QOS,
        )
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(recorder)

        sock.feed(_definition_bytes(hand="sideways"))
        assert _spin_until(executor, lambda: received, 5.0)
        assert received[0].hand == HandDescription.HAND_UNKNOWN
        assert node._warned_handedness == {7}

        node.destroy_node()
        recorder.destroy_node()
    finally:
        rclpy.shutdown()


def test_an_unset_hand_field_is_unknown_not_left():
    assert HandDescription().hand == HandDescription.HAND_UNKNOWN
    assert HandJoints().hand == HandJoints.HAND_UNKNOWN
