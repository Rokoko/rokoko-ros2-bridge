import math

from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from visualization_msgs.msg import Marker

from rkk_hand_bridge import hand_markers

_JOINT_NAMES = (
    "palm", "wrist",
    "thumb_metacarpal", "thumb_proximal", "thumb_distal", "thumb_tip",
    "index_metacarpal", "index_proximal", "index_intermediate", "index_distal", "index_tip",
    "middle_metacarpal", "middle_proximal", "middle_intermediate", "middle_distal", "middle_tip",
    "ring_metacarpal", "ring_proximal", "ring_intermediate", "ring_distal", "ring_tip",
    "little_metacarpal", "little_proximal", "little_intermediate", "little_distal", "little_tip",
)
_IDENTITY = (0.0, 0.0, 0.0, 1.0)


def _rebased(names=_JOINT_NAMES):
    # Distinct positions, so a wrongly-indexed endpoint is visible.
    return [((float(i), 0.0, 0.0), _IDENTITY) for i in range(len(names))]


def _build(names=_JOINT_NAMES, radii=None, hand="right"):
    radii = [0.01] * len(names) if radii is None else radii
    return hand_markers.build(
        hand, names, radii, _rebased(names), "world", TimeMsg(sec=1), DurationMsg(sec=1)
    )


def _spheres(array):
    return [m for m in array.markers if m.type == Marker.SPHERE]


def _bones(array):
    return next(m for m in array.markers if m.type == Marker.LINE_LIST)


def test_bone_table_follows_the_openxr_hierarchy():
    # 4 fingers x 5 bones (wrist->metacarpal->proximal->intermediate->
    # distal->tip) + the thumb's 4, which has no intermediate.
    assert len(hand_markers.BONES) == 24
    assert ("wrist", "thumb_metacarpal") in hand_markers.BONES
    assert ("thumb_proximal", "thumb_distal") in hand_markers.BONES
    assert not any("thumb_intermediate" in bone for bone in hand_markers.BONES)
    assert ("index_intermediate", "index_distal") in hand_markers.BONES
    # Every finger hangs off the wrist, not off the palm or each other.
    parents = {parent for parent, _ in hand_markers.BONES}
    assert sum(1 for parent, _ in hand_markers.BONES if parent == "wrist") == 5
    assert "palm" not in parents


def test_one_sphere_per_joint_plus_one_bone_marker():
    array = _build()
    assert len(array.markers) == len(_JOINT_NAMES) + 1
    assert len(_spheres(array)) == len(_JOINT_NAMES)
    assert {m.id for m in _spheres(array)} == set(range(len(_JOINT_NAMES)))


def test_spheres_are_diameters_of_the_reported_radii():
    radii = [0.011] * len(_JOINT_NAMES)
    sphere = _spheres(_build(radii=radii))[0]
    assert math.isclose(sphere.scale.x, 0.022)
    assert sphere.scale.x == sphere.scale.y == sphere.scale.z


def test_zero_radius_is_floored_so_rviz_still_draws_it():
    sphere = _spheres(_build(radii=[0.0] * len(_JOINT_NAMES)))[0]
    assert sphere.scale.x > 0.0


def test_every_marker_carries_a_valid_quaternion():
    # An all-zero quaternion is invalid and makes RViz log about it.
    for marker in _build().markers:
        q = marker.pose.orientation
        assert math.isclose(math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2), 1.0)


def test_bones_connect_the_positions_of_their_two_joints():
    array = _build()
    bones = _bones(array)
    assert len(bones.points) == 2 * len(hand_markers.BONES)

    index_of = {name: i for i, name in enumerate(_JOINT_NAMES)}
    start, end = bones.points[0], bones.points[1]
    parent, child = hand_markers.BONES[0]
    assert start.x == float(index_of[parent])
    assert end.x == float(index_of[child])


def test_a_reduced_joint_set_skips_bones_it_cannot_draw():
    names = ("wrist", "index_metacarpal", "index_proximal")
    array = _build(names=names)
    assert len(_spheres(array)) == 3
    # wrist->index_metacarpal->index_proximal are drawable; the rest are not.
    assert len(_bones(array).points) == 4


def test_left_and_right_hands_are_told_apart_by_colour_and_namespace():
    left, right = _build(hand="left"), _build(hand="right")
    assert _spheres(left)[0].ns != _spheres(right)[0].ns
    assert "left" in _spheres(left)[0].ns
    assert _spheres(left)[0].color != _spheres(right)[0].color


def test_deletion_clears_both_namespaces_for_one_hand():
    array = hand_markers.deletion("left")
    assert [m.action for m in array.markers] == [Marker.DELETEALL, Marker.DELETEALL]
    assert all("left" in m.ns for m in array.markers)
    assert len({m.ns for m in array.markers}) == 2
