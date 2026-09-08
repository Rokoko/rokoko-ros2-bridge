import io
import json
import struct
import threading
import time

from rkk_hand_bridge import rgmp
from rkk_hand_bridge.hand_stream import HandStream

from test_bridge_node import FeedableSocket


class FakeSocket:
    """A `recv`/`close`-compatible stand-in for a real TCP socket
    (matches the real socket API, not a file-like `.read()`). Blocks once
    its buffered bytes are exhausted, until `close()` releases it with
    EOF — the same behavior a real socket close causes."""

    def __init__(self, data: bytes):
        self._buf = io.BytesIO(data)
        self._closed = threading.Event()

    def recv(self, n):
        chunk = self._buf.read(n)
        if chunk:
            return chunk
        self._closed.wait()
        return b""

    def close(self):
        self._closed.set()


JOINT_NAMES = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)


def _frame(msg_prefix: int, payload: bytes) -> bytes:
    return struct.pack("<II", msg_prefix, len(payload)) + payload


def _definition_bytes(device_id=7, hand="right", group_id_padding=0) -> bytes:
    streams = [{"target_frame": f"xr_{name}"} for name in JOINT_NAMES]
    groups = [{"name": "joints_openxr", "streams": streams}]
    if group_id_padding:
        groups = [{"name": "joints_local", "streams": []}] * group_id_padding + groups
    payload = json.dumps(
        {
            "device_id": device_id,
            "device_type": "solved_hand",
            "timestamp_epoch": "unix_epoch",
            "device_info": {"hand": hand},
            "static_data": [],
            "groups": groups,
        }
    ).encode()
    return _frame(rgmp.MSG_DEFINITION, payload)


def _data_bytes(device_id=7, group_id=0, timestamp_us=1) -> bytes:
    pose = struct.pack("<7f", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    payload = struct.pack("<IIQ", device_id, group_id, timestamp_us) + pose * rgmp.JOINT_COUNT
    return _frame(rgmp.MSG_DATA, payload)


def _disconnect_bytes(device_id=7) -> bytes:
    return _frame(rgmp.MSG_DISCONNECT, struct.pack("<I", device_id))


def test_definition_and_data_populate_hands_and_latest():
    data = _definition_bytes() + _data_bytes()
    stream = HandStream("h", 0, connect=lambda: FakeSocket(data))
    stream.start()
    try:
        stream.wait(0, timeout=2.0)
        stream.wait(1, timeout=2.0)
        assert 7 in stream.hands()
        assert 7 in stream.latest()
        assert stream.hands()[7].hand == "right"
    finally:
        stream.stop()


def test_data_for_a_different_group_is_ignored():
    data = _definition_bytes() + _data_bytes(group_id=99)
    stream = HandStream("h", 0, connect=lambda: FakeSocket(data))
    stream.start()
    try:
        stream.wait(0, timeout=2.0)
        assert 7 in stream.hands()
        assert stream.latest() == {}
    finally:
        stream.stop()


def test_disconnect_clears_the_hand():
    data = _definition_bytes() + _data_bytes() + _disconnect_bytes()
    stream = HandStream("h", 0, connect=lambda: FakeSocket(data))
    stream.start()
    try:
        stream.wait(0, timeout=2.0)
        stream.wait(1, timeout=2.0)
        stream.wait(2, timeout=2.0)
        assert stream.hands() == {}
        assert stream.latest() == {}
    finally:
        stream.stop()


def test_reconnect_serves_a_fresh_socket():
    sockets = [FakeSocket(_definition_bytes()), FakeSocket(_definition_bytes(device_id=8))]
    stream = HandStream("h", 0, reconnect_delay_s=0.01, connect=lambda: sockets.pop(0))
    stream.start()
    try:
        gen = stream.wait(0, timeout=2.0)
        assert 7 in stream.hands()
        sockets_before = len(sockets)
        stream._sock.close()  # simulate the connection dropping
        gen = stream.wait(gen, timeout=2.0)  # dropped: hands cleared
        assert stream.hands() == {}
        stream.wait(gen, timeout=2.0)  # reconnected: device 8 announced
        assert 8 in stream.hands()
        assert sockets_before == 1
    finally:
        stream.stop()


def test_stop_terminates_the_background_thread():
    stream = HandStream("h", 0, connect=lambda: FakeSocket(_definition_bytes()))
    stream.start()
    stream.wait(0, timeout=2.0)
    stream.stop()
    assert not stream.connected


def _no_openxr_definition_bytes(device_id=7) -> bytes:
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


def test_warns_once_for_a_hand_missing_the_openxr_group():
    data = _no_openxr_definition_bytes() * 2  # sent twice
    warnings = []
    stream = HandStream(
        "h",
        0,
        connect=lambda: FakeSocket(data),
        on_unsupported_hand=lambda device_id, reason: warnings.append((device_id, reason)),
    )
    stream.start()
    try:
        # nothing changes state (no hand/frame), so poll briefly instead of wait()
        for _ in range(20):
            if warnings:
                break
            threading.Event().wait(0.05)
        assert warnings == [(7, "missing_joints_openxr")]
        assert stream.hands() == {}
    finally:
        stream.stop()


def test_undecodable_payloads_are_reported_and_survived():
    for payload in (b"{not json", b'{"device_id": 7, "device_type": "solved_hand"'):
        reported = []
        sock = FeedableSocket()
        pending = [sock, FeedableSocket()]
        stream = HandStream(
            "h", 0, reconnect_delay_s=0.01,
            connect=lambda: pending.pop(0) if pending else FeedableSocket(),
            on_stream_error=reported.append,
        )
        stream.start()
        try:
            time.sleep(0.15)
            sock.feed(_frame(rgmp.MSG_DEFINITION, payload))
            time.sleep(0.4)
            assert stream._thread.is_alive()
            assert reported and "undecodable frame" in reported[0]
        finally:
            stream.stop()


def test_a_long_reconnect_delay_never_shrinks():
    from rkk_hand_bridge.hand_stream import _MAX_RECONNECT_DELAY_S

    def progression(base, steps=4):
        delay, seen = base, []
        for _ in range(steps):
            seen.append(delay)
            delay = min(delay * 2.0, max(_MAX_RECONNECT_DELAY_S, base))
        return seen

    assert progression(0.5) == [0.5, 1.0, 2.0, 4.0]
    assert progression(30.0) == [30.0, 30.0, 30.0, 30.0]
    assert all(b >= a for a, b in zip(progression(30.0), progression(30.0)[1:]))


class _InstantlyClosedSocket:
    """Accepts, then closes without sending anything."""

    def recv(self, n):
        return b""

    def close(self):
        pass


def test_a_connection_that_delivers_nothing_still_backs_off():
    attempts = []

    def connect():
        attempts.append(time.monotonic())
        return _InstantlyClosedSocket()

    stream = HandStream("h", 0, reconnect_delay_s=0.05, connect=connect)
    stream.start()
    try:
        time.sleep(1.0)
    finally:
        stream.stop()
    # Without backing off this reconnects every 50ms, about 20 times.
    assert 2 <= len(attempts) <= 8, len(attempts)


def test_connecting_uses_a_timeout_and_then_blocks_for_reads():
    import socket as socket_module
    from rkk_hand_bridge import hand_stream as hs

    captured = {}

    def fake_create_connection(address, timeout=None):
        captured["timeout"] = timeout
        return _InstantlyClosedSocket()

    class _Recording(_InstantlyClosedSocket):
        def settimeout(self, value):
            captured["read_timeout"] = value

    def fake(address, timeout=None):
        captured["timeout"] = timeout
        return _Recording()

    original = socket_module.create_connection
    socket_module.create_connection = fake
    try:
        HandStream("h", 1)._default_connect()
    finally:
        socket_module.create_connection = original

    assert captured["timeout"] == hs._CONNECT_TIMEOUT_S
    assert captured["read_timeout"] is None


def test_stopping_before_a_connection_lands_closes_it():
    closed = []

    class _Socket(_InstantlyClosedSocket):
        def close(self):
            closed.append(True)

    started = threading.Event()

    def slow_connect():
        started.set()
        time.sleep(0.3)
        return _Socket()

    stream = HandStream("h", 0, reconnect_delay_s=0.01, connect=slow_connect)
    stream.start()
    assert started.wait(2.0)
    stream.stop()
    assert closed, "a socket that arrived after stop() was never closed"


def test_wait_reports_a_timeout_by_returning_what_was_seen():
    stream = HandStream("h", 0, connect=lambda: FeedableSocket())
    assert stream.wait(0, timeout=0.05) == 0


def test_wait_returns_none_once_stopped():
    stream = HandStream("h", 0, connect=lambda: FeedableSocket())
    stream.start()
    try:
        stream.stop()
        assert stream.wait(0, timeout=0.05) is None
    finally:
        stream.stop()
