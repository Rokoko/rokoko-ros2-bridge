import io
import json
import struct

import pytest

from rkk_hand_bridge import rgmp


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


def _openxr_group(names=JOINT_NAMES):
    return {"name": "joints_openxr",
            "streams": [{"target_frame": f"xr_{name}"} for name in names]}


def test_decode_definition_extracts_joints_and_radii():
    groups = [{"name": "joints_local", "streams": []}, _openxr_group()]
    static_data = [
        {"custom_label": "joint_radius", "target_frame": "xr_palm", "value": 0.025},
    ]
    definition = rgmp.decode_definition(_definition(groups=groups, static_data=static_data))

    assert definition.device_id == 7
    assert definition.hand == "right"
    assert definition.timestamp_epoch == "unix_epoch"
    assert definition.group_id == 1
    assert definition.joint_names == JOINT_NAMES
    assert definition.joint_radii[0] == 0.025
    assert definition.joint_radii[1] == 0.0


def test_a_group_with_the_wrong_joint_count_is_rejected():
    # The published messages are fixed-size arrays; a short one would be
    # accepted here and then crash the publisher.
    groups = [_openxr_group(JOINT_NAMES[:20])]
    payload = _definition(groups=groups)
    assert rgmp.decode_definition(payload) is None
    assert rgmp.describe_unsupported(payload).startswith(rgmp.WRONG_JOINT_COUNT)


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


def test_a_group_that_states_its_id_is_believed_over_its_index():
    group = _openxr_group()
    group["group_id"] = 9
    definition = rgmp.decode_definition(_definition(groups=[{"name": "x"}, group]))
    assert definition.group_id == 9
