import io
import json
import struct
import threading

from rkk_hand_bridge import rgmp_client as rgmp
from rkk_hand_bridge.hand_stream import HandStream


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


def _frame(msg_prefix: int, payload: bytes) -> bytes:
    return struct.pack("<II", msg_prefix, len(payload)) + payload


def _definition_bytes(device_id=7, hand="right", group_id_padding=0) -> bytes:
    groups = [{"name": "joints_openxr", "streams": [{"target_frame": "xr_wrist"}]}]
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
    stream = HandStream("h", 0, reconnect_backoff_s=0.01, connect=lambda: sockets.pop(0))
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
        on_unusable_hand=lambda device_id, reason: warnings.append((device_id, reason)),
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
