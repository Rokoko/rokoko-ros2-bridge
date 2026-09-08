import math

import pytest

from rkk_hand_bridge import frame_convert as fc


def _close(a, b, tol=1e-9):
    return math.isclose(a, b, abs_tol=tol)


def test_xr_base_to_ros_maps_y_up_to_z_up():
    x, y, z = fc.q_rotate(fc.XR_BASE_TO_ROS, (0.0, 1.0, 0.0))
    assert _close(x, 0.0) and _close(y, 0.0) and _close(z, 1.0)


def test_xr_base_to_ros_leaves_x_unchanged():
    v = fc.q_rotate(fc.XR_BASE_TO_ROS, (1.0, 0.0, 0.0))
    assert all(_close(a, b) for a, b in zip(v, (1.0, 0.0, 0.0)))


def test_default_converter_has_zero_yaw():
    assert fc.XrToRosConverter().yaw_rad == 0.0
    assert fc.XrToRosConverter().rotation() == fc.XR_BASE_TO_ROS


def test_measure_heading_of_identity_orientation():
    # forward (-Z) rotates through the fixed conversion alone to +Y: heading pi/2.
    assert _close(fc.measure_heading(fc.IDENTITY), math.pi / 2.0)


def test_measure_heading_raises_when_too_vertical():
    straight_up = fc.axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)  # forward -> +Z
    with pytest.raises(ValueError):
        fc.measure_heading(straight_up)


def test_capture_yaw_offset_and_apply_align_to_facing():
    offset = fc.capture_yaw_offset(fc.IDENTITY, facing_rad=0.0)
    converter = fc.XrToRosConverter(yaw_rad=offset)
    _, oriented = converter.convert((0.0, 0.0, 0.0), fc.IDENTITY)
    fx, fy, _ = fc.q_rotate(oriented, fc._WRIST_FORWARD)
    assert _close(math.atan2(fy, fx), 0.0, tol=1e-6)


def test_convert_only_rotates_no_translation():
    position, _ = fc.XrToRosConverter().convert((1.0, 2.0, 3.0), fc.IDENTITY)
    assert position != (1.0, 2.0, 3.0)  # rotated
    origin, _ = fc.XrToRosConverter().convert((0.0, 0.0, 0.0), fc.IDENTITY)
    assert origin == (0.0, 0.0, 0.0)  # no anchor offset added


def test_heading_spread_of_one_heading_is_zero():
    assert fc.heading_spread([1.2]) == 0.0


def test_heading_spread_measures_the_widest_gap():
    assert math.isclose(fc.heading_spread([0.0, 0.3, -0.2]), 0.5)


def test_heading_spread_wraps_across_pi():
    # 175 degrees and -175 degrees are 10 degrees apart, not 350.
    spread = fc.heading_spread([math.radians(175), math.radians(-175)])
    assert math.isclose(spread, math.radians(10), abs_tol=1e-9)


def test_heading_spread_never_exceeds_pi():
    headings = [0.0, math.radians(170), math.radians(-170)]
    assert fc.heading_spread(headings) <= math.pi + 1e-9


def test_heading_spread_is_independent_of_ordering():
    headings = [math.radians(10), math.radians(-25), math.radians(40)]
    assert math.isclose(fc.heading_spread(headings),
                        fc.heading_spread(list(reversed(headings))))
