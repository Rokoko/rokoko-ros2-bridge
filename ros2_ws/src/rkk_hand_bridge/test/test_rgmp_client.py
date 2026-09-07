import io
import json
import struct

import pytest

from rkk_hand_bridge import rgmp_client as rgmp


class _RecvSocket:
    """A `.recv`-compatible stand-in for a real socket, backed by bytes
    already in hand - matches the real socket API, unlike a bare BytesIO."""

    def __init__(self, data: bytes):
        self._buf = io.BytesIO(data)

    def recv(self, n):
        return self._buf.read(n)


def test_read_frame_splits_header_and_payload():
    payload = b"hello"
    sock = _RecvSocket(struct.pack("<II", 1, len(payload)) + payload)
    msg_prefix, body = rgmp.read_frame(sock)
    assert msg_prefix == 1
    assert body == payload


def test_read_frame_raises_on_truncated_stream():
    with pytest.raises(EOFError):
        rgmp.read_frame(_RecvSocket(b"\x00\x00"))


def _definition(device_type="solved_hand", groups=None, static_data=None, hand="right"):
    return json.dumps(
        {
            "device_id": 7,
            "device_type": device_type,
            "timestamp_epoch": "unix_epoch",
            "device_info": {"hand": hand},
            "static_data": static_data or [],
            "groups": groups if groups is not None else [],
        }
    ).encode()


def test_decode_definition_ignores_non_solved_hand_devices():
    assert rgmp.decode_definition(_definition(device_type="smartgloves")) is None


def test_decode_definition_ignores_missing_openxr_group():
    groups = [{"name": "joints_local", "streams": []}]
    assert rgmp.decode_definition(_definition(groups=groups)) is None


def test_decode_definition_extracts_joints_and_radii():
    groups = [
        {"name": "joints_local", "streams": []},
        {
            "name": "joints_openxr",
            "streams": [
                {"target_frame": "xr_palm"},
                {"target_frame": "xr_wrist"},
            ],
        },
    ]
    static_data = [
        {"custom_label": "joint_radius", "target_frame": "xr_palm", "value": 0.025},
    ]
    definition = rgmp.decode_definition(_definition(groups=groups, static_data=static_data))

    assert definition.device_id == 7
    assert definition.hand == "right"
    assert definition.timestamp_epoch == "unix_epoch"
    assert definition.group_id == 1
    assert definition.joint_names == ("palm", "wrist")
    assert definition.joint_radii == (0.025, 0.0)


def _pose_bytes(position, orientation):
    return struct.pack("<7f", *position, *orientation)


def test_decode_data_reads_header_and_all_joints():
    positions = [(float(i), 0.0, 0.0) for i in range(rgmp.JOINT_COUNT)]
    orientation = (0.0, 0.0, 0.0, 1.0)
    payload = struct.pack("<IIQ", 7, 1, 1_234_567)
    for position in positions:
        payload += _pose_bytes(position, orientation)

    frame = rgmp.decode_data(payload)

    assert frame.device_id == 7
    assert frame.group_id == 1
    assert frame.timestamp_us == 1_234_567
    assert len(frame.joints) == rgmp.JOINT_COUNT
    assert frame.joints[3].position == (3.0, 0.0, 0.0)
    assert frame.joints[3].orientation == orientation


def test_decode_disconnect_extracts_device_id():
    assert rgmp.decode_disconnect(struct.pack("<I", 42)).device_id == 42
