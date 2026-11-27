import math

import pytest

from odom_drive import (POSE_TIMEOUT_S, NewPoses, PoseTracker, moves_for,
                        ramped_speed, yaw_from_quaternion)


def test_distance_is_in_a_straight_line_not_along_the_wobble():
    tracker = PoseTracker()
    for x, y in ((0.0, 0.0), (0.3, 0.4), (0.0, 0.8), (0.3, 0.4)):
        tracker.feed(x, y, 0.0)
    assert tracker.distance_from((0.0, 0.0, 0.0)) == pytest.approx(0.5)


def test_counts_whole_turns_across_the_wrap_at_pi():
    tracker = PoseTracker()
    for degrees in range(0, 721, 10):
        tracker.feed(0.0, 0.0, math.remainder(math.radians(degrees),
                                              2 * math.pi))
    assert math.degrees(tracker.turn_rad) == pytest.approx(720.0)


def test_spinning_right_counts_negative():
    tracker = PoseTracker()
    for degrees in range(0, -361, -10):
        tracker.feed(0.0, 0.0, math.remainder(math.radians(degrees),
                                              2 * math.pi))
    assert math.degrees(tracker.turn_rad) == pytest.approx(-360.0)


def test_offset_is_along_the_start_heading():
    tracker = PoseTracker()
    tracker.feed(1.0, 1.0, math.pi / 2)  # facing +y
    tracker.feed(1.0, 2.0, math.pi / 2)
    tracker.feed(0.9, 2.0, math.pi / 2)
    assert tracker.offset_from_start() == pytest.approx((1.0, 0.1))


def test_yaw_from_a_quaternion_about_z():
    half = math.radians(30) / 2
    assert yaw_from_quaternion(math.sin(half), math.cos(half)) == \
        pytest.approx(math.radians(30))


def test_one_move_for_a_straight_run_or_a_spin():
    assert moves_for("straight", -0.5) == [("straight", -0.5)]
    assert moves_for("spin", 720) == [("spin", 720)]


def test_a_square_turns_the_way_its_sign_says():
    left = moves_for("square", 1.0)
    right = moves_for("square", -1.0)
    assert left == [("straight", 1.0), ("spin", 90.0)] * 4
    assert right == [("straight", 1.0), ("spin", -90.0)] * 4


def test_speeds_up_no_faster_than_the_acceleration_allows():
    assert ramped_speed(0.0, 1.0, 0.5, 3.2, 0.02) == pytest.approx(0.064)


def test_holds_cruise_speed_mid_move():
    assert ramped_speed(0.5, 1.0, 0.5, 3.2, 0.02) == pytest.approx(0.5)


def test_slows_so_it_could_stop_in_what_remains():
    # 0.025 rad left at 3.2 rad/s^2: no faster than sqrt(2 * 3.2 * 0.025).
    assert ramped_speed(0.5, 0.025, 0.5, 3.2, 0.02) == pytest.approx(0.4)


def test_keeps_a_fifth_of_cruise_until_the_end():
    assert ramped_speed(0.2, 1e-6, 0.5, 3.2, 0.02) == pytest.approx(0.1)


def test_feeds_each_stamp_once():
    tracker = PoseTracker()
    poses = NewPoses(tracker)
    poses.feed((1, 0), 0.0, 0.0, 0.0, now_s=10.0)
    poses.feed((1, 0), 5.0, 5.0, 1.0, now_s=10.1)  # the same pose, read again
    assert tracker.last == (0.0, 0.0, 0.0)


def test_a_pose_that_stops_updating_goes_stale():
    poses = NewPoses(PoseTracker())
    assert poses.is_stale(now_s=0.0)
    poses.feed((1, 0), 0.0, 0.0, 0.0, now_s=10.0)
    poses.feed((1, 0), 0.0, 0.0, 0.0, now_s=11.0)  # frozen: same stamp
    assert not poses.is_stale(now_s=10.0 + POSE_TIMEOUT_S / 2)
    assert poses.is_stale(now_s=11.0)
