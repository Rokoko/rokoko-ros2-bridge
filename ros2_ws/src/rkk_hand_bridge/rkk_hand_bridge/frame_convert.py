"""Converting solved-hand poses from xr_base (Y-up) to ROS (REP-103, Z-up)."""

import math
from dataclasses import dataclass

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]  # x, y, z, w

IDENTITY: Quat = (0.0, 0.0, 0.0, 1.0)

UP_AXIS: Vec3 = (0.0, 0.0, 1.0)

# OpenXR points +Z backward along the bone, so forward is -Z.
_WRIST_FORWARD: Vec3 = (0.0, 0.0, -1.0)
_MIN_HORIZONTAL = 0.2


def axis_angle(axis: Vec3, angle_rad: float) -> Quat:
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    s = math.sin(angle_rad / 2.0) / norm
    return (ax * s, ay * s, az * s, math.cos(angle_rad / 2.0))


# Y-up to Z-up: +90 degrees about X, leaving X untouched.
XR_BASE_TO_ROS: Quat = axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)


def q_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product: rotating by the result applies b first, then a."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_normalize(q: Quat) -> Quat:
    x, y, z, w = q
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return (x / norm, y / norm, z / norm, w / norm)


def q_rotate(q: Quat, v: Vec3) -> Vec3:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def normalize_angle(rad: float) -> float:
    """Wrap into (-pi, pi]."""
    wrapped = math.fmod(rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


@dataclass(frozen=True)
class XrToRosConverter:
    # A yaw of 0.0 is uncalibrated but correctly shaped, so data can
    # publish before ~/calibrate has given the yaw real meaning.
    yaw_rad: float = 0.0

    def rotation(self) -> Quat:
        yaw = axis_angle(UP_AXIS, self.yaw_rad)
        return q_normalize(q_mul(yaw, XR_BASE_TO_ROS))

    def convert(self, position: Vec3, orientation: Quat) -> tuple[Vec3, Quat]:
        return _apply(self.rotation(), position, orientation)

    def convert_all(self, poses) -> list[tuple[Vec3, Quat]]:
        rotation = self.rotation()
        return [_apply(rotation, p.position, p.orientation) for p in poses]


def _apply(rotation: Quat, position: Vec3, orientation: Quat) -> tuple[Vec3, Quat]:
    return q_rotate(rotation, position), q_normalize(q_mul(rotation, orientation))


def measure_heading(wrist_orientation: Quat) -> float:
    """Current heading of the wrist's forward direction, before any yaw
    calibration. Raises ValueError if the hand is too vertical for a
    horizontal heading to mean anything."""
    q = q_normalize(q_mul(XR_BASE_TO_ROS, wrist_orientation))
    fx, fy, fz = q_rotate(q, _WRIST_FORWARD)
    if math.hypot(fx, fy) < _MIN_HORIZONTAL:
        raise ValueError("hand is too vertical to read a heading")
    return math.atan2(fy, fx)


def capture_yaw_offset(wrist_orientation: Quat, facing_rad: float = 0.0) -> float:
    """The yaw offset that makes the wrist face `facing_rad`."""
    return yaw_offset_for_heading(measure_heading(wrist_orientation), facing_rad)


def yaw_offset_for_heading(heading_rad: float, facing_rad: float = 0.0) -> float:
    return normalize_angle(facing_rad - heading_rad)


def heading_spread(headings: list[float]) -> float:
    """Widest angle between any two headings, wrapping correctly. Never
    exceeds pi, since beyond that they are closer the other way round."""
    widest = 0.0
    for i, first in enumerate(headings):
        for second in headings[i + 1:]:
            widest = max(widest, abs(normalize_angle(second - first)))
    return widest
