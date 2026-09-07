"""RGMP v2 framing and solved-hand payload decoding."""

import json
import struct
from dataclasses import dataclass

MSG_DEFINITION = 1
MSG_DATA = 2
MSG_DISCONNECT = 3

DEVICE_TYPE = "solved_hand"
GROUP_NAME = "joints_openxr"
JOINT_COUNT = 26
RADIUS_LABEL = "joint_radius"
XR_PREFIX = "xr_"

_POSE_SIZE = 28  # FLOAT[7]: position + quaternion, 4 bytes each
_DATA_HEADER_SIZE = 16  # device_id, group_id, timestamp_us


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]  # x, y, z, w


@dataclass(frozen=True)
class Definition:
    device_id: int
    hand: str
    timestamp_epoch: str | None
    group_id: int
    joint_names: tuple[str, ...]
    joint_radii: tuple[float, ...]


@dataclass(frozen=True)
class DataFrame:
    device_id: int
    group_id: int
    timestamp_us: int
    joints: tuple[Pose, ...]


@dataclass(frozen=True)
class Disconnect:
    device_id: int


def read_frame(reader) -> tuple[int, bytes]:
    msg_prefix, msg_len = struct.unpack("<II", _read_exact(reader, 8))
    return msg_prefix, _read_exact(reader, msg_len)


def _read_exact(reader, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = reader.read(n - len(buf))
        if not chunk:
            raise EOFError("connection closed")
        buf.extend(chunk)
    return bytes(buf)


def decode_definition(payload: bytes) -> Definition | None:
    definition = json.loads(payload)
    if definition.get("device_type") != DEVICE_TYPE:
        return None

    group_id, group = _find_group(definition.get("groups") or [])
    if group is None:
        return None

    joint_names = tuple(
        stream["target_frame"].removeprefix(XR_PREFIX) for stream in group["streams"]
    )
    radii = _radii_by_joint(definition.get("static_data") or [])

    info = definition.get("device_info") or {}
    return Definition(
        device_id=definition["device_id"],
        hand=info.get("hand", "unknown"),
        timestamp_epoch=definition.get("timestamp_epoch"),
        group_id=group_id,
        joint_names=joint_names,
        joint_radii=tuple(radii.get(name, 0.0) for name in joint_names),
    )


def _find_group(groups: list) -> tuple[int, dict | None]:
    for group_id, group in enumerate(groups):
        if group.get("name") == GROUP_NAME:
            return group_id, group
    return -1, None


def _radii_by_joint(static_data: list) -> dict:
    radii = {}
    for entry in static_data:
        if entry.get("custom_label") == RADIUS_LABEL:
            name = entry["target_frame"].removeprefix(XR_PREFIX)
            radii[name] = entry["value"]
    return radii


def decode_data_header(payload: bytes) -> tuple[int, int, int]:
    return struct.unpack_from("<IIQ", payload, 0)


def decode_data(payload: bytes) -> DataFrame:
    device_id, group_id, timestamp_us = decode_data_header(payload)
    joints = tuple(
        _decode_pose(payload, _DATA_HEADER_SIZE + i * _POSE_SIZE)
        for i in range(JOINT_COUNT)
    )
    return DataFrame(device_id, group_id, timestamp_us, joints)


def _decode_pose(payload: bytes, offset: int) -> Pose:
    x, y, z, qx, qy, qz, qw = struct.unpack_from("<7f", payload, offset)
    return Pose((x, y, z), (qx, qy, qz, qw))


def decode_disconnect(payload: bytes) -> Disconnect:
    (device_id,) = struct.unpack("<I", payload)
    return Disconnect(device_id)
