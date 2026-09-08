"""MarkerArray rendering of a solved hand, for RViz.

TF alone draws 26 identical axis triads per hand, which reads as
coordinate-frame soup rather than a hand. This turns the same rebased
poses into a sphere per joint (sized from the description's joint radii)
plus a LINE_LIST skeleton, which reads as a hand at a glance.

The bone table is the fixed XrHandJointEXT hierarchy, so it is a static
lookup here rather than something read off the wire.
"""

from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from rkk_hand_bridge.frame_convert import Quat, Vec3

# Thumbs have no intermediate phalanx; every other finger does.
_FINGER_PARTS = ("metacarpal", "proximal", "intermediate", "distal", "tip")
_THUMB_PARTS = ("metacarpal", "proximal", "distal", "tip")
_FINGERS = ("thumb", "index", "middle", "ring", "little")


def _finger_bones(finger: str) -> list[tuple[str, str]]:
    parts = _THUMB_PARTS if finger == "thumb" else _FINGER_PARTS
    names = [f"{finger}_{part}" for part in parts]
    # wrist -> metacarpal, then straight down the chain to the tip.
    return list(zip(["wrist"] + names, names))


BONES: tuple[tuple[str, str], ...] = tuple(
    bone for finger in _FINGERS for bone in _finger_bones(finger)
)

_HAND_COLORS = {
    "left": (0.35, 0.62, 0.95),
    "right": (0.96, 0.60, 0.26),
}
_FALLBACK_COLOR = (0.7, 0.7, 0.7)

# RViz drops a zero-scaled marker and logs about it, so keep a floor
# under radii the description may report as 0.
_MIN_RADIUS_M = 0.002
_BONE_WIDTH_M = 0.004

# OpenXR reports the wrist and palm at their true anatomical radii, which
# are several times a fingertip's. Drawn to scale they swamp the fingers
# and the hand looks swollen, so cap the drawn size. A sphere's *centre*
# is still the exact joint position either way - capping costs no
# positional accuracy, only the (unhelpful) illusion of volume.
_DEFAULT_MAX_RADIUS_M = 0.010

_SPHERES_NS = "joints"
_BONES_NS = "bones"


def _identity_pose() -> Pose:
    # An all-zero quaternion is invalid and makes RViz complain; markers
    # that carry no rotation of their own still need w=1.
    return Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))


def _color(hand: str, alpha: float) -> ColorRGBA:
    r, g, b = _HAND_COLORS.get(hand, _FALLBACK_COLOR)
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def _namespace(hand: str, kind: str) -> str:
    return f"rkk_{hand}_hand/{kind}"


def build(
    hand: str,
    joint_names,
    joint_radii,
    rebased: list[tuple[Vec3, Quat]],
    frame_id: str,
    stamp,
    lifetime,
    scale: float = 1.0,
    max_radius_m: float = _DEFAULT_MAX_RADIUS_M,
) -> MarkerArray:
    """One sphere per joint plus a single LINE_LIST of bones.

    `rebased` is the same (position, orientation) list published as
    HandJoints and TF, so markers cost no extra maths and cannot drift
    out of step with them. Joints named in BONES but absent from
    `joint_names` are skipped rather than raising - a description with a
    reduced joint set still renders what it does have.

    `scale` shrinks every sphere; `max_radius_m` caps the big ones. Both
    affect drawn size only, never position.
    """
    array = MarkerArray()
    index_of = {name: i for i, name in enumerate(joint_names)}

    for i, (name, (position, _orientation)) in enumerate(zip(joint_names, rebased)):
        radius = joint_radii[i] if i < len(joint_radii) else 0.0
        radius = min(radius * scale, max_radius_m) if max_radius_m > 0.0 else radius * scale
        diameter = 2.0 * max(radius, _MIN_RADIUS_M)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = _namespace(hand, _SPHERES_NS)
        marker.id = i
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = _identity_pose()
        marker.pose.position = Point(x=position[0], y=position[1], z=position[2])
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color = _color(hand, 0.95)
        marker.lifetime = lifetime
        marker.frame_locked = True
        marker.text = name  # shows up in RViz's marker inspector
        array.markers.append(marker)

    bones = Marker()
    bones.header.frame_id = frame_id
    bones.header.stamp = stamp
    bones.ns = _namespace(hand, _BONES_NS)
    bones.id = 0
    bones.type = Marker.LINE_LIST
    bones.action = Marker.ADD
    bones.pose = _identity_pose()
    bones.scale.x = _BONE_WIDTH_M
    bones.color = _color(hand, 0.85)
    bones.lifetime = lifetime
    bones.frame_locked = True
    for parent, child in BONES:
        if parent not in index_of or child not in index_of:
            continue
        for endpoint in (index_of[parent], index_of[child]):
            position = rebased[endpoint][0]
            bones.points.append(Point(x=position[0], y=position[1], z=position[2]))
    array.markers.append(bones)

    return array


def deletion(hand: str) -> MarkerArray:
    """DELETEALL for one hand's namespaces, so a disconnect clears RViz
    immediately instead of waiting out the markers' lifetime."""
    array = MarkerArray()
    for kind in (_SPHERES_NS, _BONES_NS):
        marker = Marker()
        marker.ns = _namespace(hand, kind)
        marker.action = Marker.DELETEALL
        array.markers.append(marker)
    return array
