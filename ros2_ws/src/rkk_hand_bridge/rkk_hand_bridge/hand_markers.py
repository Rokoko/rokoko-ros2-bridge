"""MarkerArray rendering for RViz of a solved hand: a sphere per joint, joined by a skeleton."""

from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from rkk_hand_bridge.frame_convert import Quat, Vec3

_FINGERS = ("thumb", "index", "middle", "ring", "little")
_PHALANGES = ("metacarpal", "proximal", "intermediate", "distal", "tip")
_THUMB_PHALANGES = ("metacarpal", "proximal", "distal", "tip")

_HAND_COLORS = {"left": (0.35, 0.62, 0.95), "right": (0.96, 0.60, 0.26)}
_FALLBACK_COLOR = (0.7, 0.7, 0.7)

_MIN_RADIUS_M = 0.002  # RViz drops a zero-scaled marker
_BONE_WIDTH_M = 0.004
# OpenXR reports true anatomical radii, so an uncapped wrist swamps the
# fingers. Centres are untouched, so only drawn size changes.
_DEFAULT_MAX_RADIUS_M = 0.010

_SPHERES_NS = "joints"
_BONES_NS = "bones"


def _finger_bones(finger: str) -> list[tuple[str, str]]:
    phalanges = _THUMB_PHALANGES if finger == "thumb" else _PHALANGES
    names = [f"{finger}_{phalanx}" for phalanx in phalanges]
    return list(zip(["wrist"] + names, names))


BONES: tuple[tuple[str, str], ...] = tuple(
    bone for finger in _FINGERS for bone in _finger_bones(finger)
)


def _identity_pose() -> Pose:
    # An all-zero quaternion is invalid; RViz logs about it.
    return Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))


def _color(hand: str) -> ColorRGBA:
    # Opaque: anything less puts 26 overlapping spheres in the renderer's
    # transparent queue, whose cost scales with the window's pixels.
    r, g, b = _HAND_COLORS.get(hand, _FALLBACK_COLOR)
    return ColorRGBA(r=r, g=g, b=b, a=1.0)


def _namespace(hand: str, kind: str) -> str:
    return f"rkk_{hand}_hand/{kind}"


def _new_marker(hand: str, kind: str, frame_id: str, stamp, lifetime) -> Marker:
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = _namespace(hand, kind)
    marker.action = Marker.ADD
    marker.pose = _identity_pose()
    marker.lifetime = lifetime
    marker.frame_locked = True
    return marker


def _drawn_diameter(radius: float, scale: float, max_radius_m: float) -> float:
    scaled = radius * scale
    capped = min(scaled, max_radius_m) if max_radius_m > 0.0 else scaled
    return 2.0 * max(capped, _MIN_RADIUS_M)


def _point(position: Vec3) -> Point:
    return Point(x=position[0], y=position[1], z=position[2])


def slot_offset(slot: int, hand_count: int, spacing_m: float) -> Vec3:
    """Fans hands out along +Y, centred on the parent frame's origin."""
    return (0.0, (slot - (hand_count - 1) / 2.0) * spacing_m, 0.0)


def build(
    hand: str,
    joint_names,
    joint_radii,
    converted: list[tuple[Vec3, Quat]],
    frame_id: str,
    stamp,
    lifetime,
    scale: float = 1.0,
    max_radius_m: float = _DEFAULT_MAX_RADIUS_M,
    offset: Vec3 = (0.0, 0.0, 0.0),
) -> MarkerArray:
    """`converted` is the (position, orientation) list also published as
    HandJoints and TF; `offset` moves the drawing only. Bones whose joints
    are absent are skipped."""
    placed = [
        ((p[0] + offset[0], p[1] + offset[1], p[2] + offset[2]), q) for p, q in converted
    ]
    array = MarkerArray()
    array.markers.extend(
        _joint_spheres(hand, joint_names, joint_radii, placed, frame_id, stamp,
                       lifetime, scale, max_radius_m)
    )
    array.markers.append(
        _bone_skeleton(hand, joint_names, placed, frame_id, stamp, lifetime)
    )
    return array


def _joint_spheres(hand, joint_names, joint_radii, converted, frame_id, stamp,
                   lifetime, scale, max_radius_m) -> list[Marker]:
    spheres = []
    for i, (name, (position, _orientation)) in enumerate(zip(joint_names, converted)):
        radius = joint_radii[i] if i < len(joint_radii) else 0.0
        marker = _new_marker(hand, _SPHERES_NS, frame_id, stamp, lifetime)
        marker.id = i
        marker.type = Marker.SPHERE
        marker.pose.position = _point(position)
        marker.scale.x = marker.scale.y = marker.scale.z = _drawn_diameter(
            radius, scale, max_radius_m
        )
        marker.color = _color(hand)
        marker.text = name
        spheres.append(marker)
    return spheres


def _bone_skeleton(hand, joint_names, converted, frame_id, stamp, lifetime) -> Marker:
    index_of = {name: i for i, name in enumerate(joint_names)}
    bones = _new_marker(hand, _BONES_NS, frame_id, stamp, lifetime)
    bones.id = 0
    bones.type = Marker.LINE_LIST
    bones.scale.x = _BONE_WIDTH_M
    bones.color = _color(hand)
    for parent, child in BONES:
        if parent in index_of and child in index_of:
            bones.points.append(_point(converted[index_of[parent]][0]))
            bones.points.append(_point(converted[index_of[child]][0]))
    return bones


def deletion(hand: str) -> MarkerArray:
    """Clears a disconnected hand without waiting out the markers' lifetime."""
    array = MarkerArray()
    for kind in (_SPHERES_NS, _BONES_NS):
        marker = Marker()
        marker.ns = _namespace(hand, kind)
        marker.action = Marker.DELETEALL
        array.markers.append(marker)
    return array
