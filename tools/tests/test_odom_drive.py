import math

import pytest

from odom_drive import OdomTracker, yaw_from_quaternion


def test_adds_up_the_distance_travelled():
    tracker = OdomTracker()
    for x in (0.0, 0.3, 0.6, 1.0):
        tracker.feed(x, 0.0, 0.0)
    assert tracker.distance_m == pytest.approx(1.0)
    assert tracker.offset_from_start() == pytest.approx((1.0, 0.0))


def test_counts_whole_turns_across_the_wrap_at_pi():
    tracker = OdomTracker()
    for degrees in range(0, 721, 10):
        tracker.feed(0.0, 0.0, math.remainder(math.radians(degrees),
                                              2 * math.pi))
    assert math.degrees(tracker.turn_rad) == pytest.approx(720.0)


def test_spinning_right_counts_negative():
    tracker = OdomTracker()
    for degrees in range(0, -361, -10):
        tracker.feed(0.0, 0.0, math.remainder(math.radians(degrees),
                                              2 * math.pi))
    assert math.degrees(tracker.turn_rad) == pytest.approx(-360.0)


def test_offset_is_along_the_start_heading():
    tracker = OdomTracker()
    tracker.feed(1.0, 1.0, math.pi / 2)  # facing +y
    tracker.feed(1.0, 2.0, math.pi / 2)
    tracker.feed(0.9, 2.0, math.pi / 2)
    assert tracker.offset_from_start() == pytest.approx((1.0, 0.1))


def test_yaw_from_a_quaternion_about_z():
    half = math.radians(30) / 2
    assert yaw_from_quaternion(math.sin(half), math.cos(half)) == \
        pytest.approx(math.radians(30))
