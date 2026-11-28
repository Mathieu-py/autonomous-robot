import math

import numpy as np
import pytest

from end_pose_errors import (FLAG_SPACING_MM, PoseChange, drives, errors,
                             floor_end_pose, moves_of, overshoots,
                             planned_turn_deg, pose_change, sample_sd)


def flag_readings(ahead_mm, left_mm, left_over_deg):
    """Where each flag ends from its start mark (along + ahead, across +
    right) for a robot that ended there, turned that much past whole turns."""
    half = FLAG_SPACING_MM / 2
    turn = math.radians(left_over_deg)
    front = (ahead_mm + half * math.cos(turn) - half, left_mm + half * math.sin(turn))
    rear = (ahead_mm - half * math.cos(turn) + half, left_mm - half * math.sin(turn))
    return front[0], -front[1], rear[0], -rear[1]


def test_reads_the_end_pose_from_the_flags():
    real = floor_end_pose(360.0, *flag_readings(3.0, -7.0, 1.5))
    assert real == pytest.approx((0.003, -0.007, 361.5))


def test_a_square_turns_once_the_way_its_sign_says():
    assert planned_turn_deg(moves_of("square 1.0")) == 360.0
    assert planned_turn_deg(moves_of("square -1.0")) == -360.0


def test_a_spin_turns_as_far_as_it_says():
    assert planned_turn_deg(moves_of("spin -720")) == -720.0


def test_a_straight_run_doesnt_turn():
    assert planned_turn_deg(moves_of("straight 1.0")) == 0.0


def test_extra_turn_counts_positive_either_way():
    left = errors(PoseChange(0.0, 0.0, 361.0), PoseChange(0.0, 0.0, 360.0),
                  360.0)
    right = errors(PoseChange(0.0, 0.0, -361.0), PoseChange(0.0, 0.0, -360.0),
                   -360.0)
    assert left.turn_error_deg == pytest.approx(1.0)
    assert right.turn_error_deg == pytest.approx(1.0)


def test_end_error_is_the_distance_between_the_two_ends():
    error = errors(PoseChange(0.003, 0.004, 0.0), PoseChange(0.0, 0.0, 0.0),
                   360.0)
    assert error.end_error_mm == pytest.approx(5.0)


def test_a_long_gap_starts_a_new_drive():
    times = np.array([0.0, 0.02, 0.04, 60.0, 60.02])
    commands = np.column_stack([times, np.zeros(5), np.zeros(5)])
    assert [list(d[:, 0]) for d in drives(commands)] == \
        [[0.0, 0.02, 0.04], [60.0, 60.02]]


def test_a_turn_past_its_target_counts_positive_on_a_right_spin():
    t = np.arange(0.0, 10.0, 0.02)
    commands = np.column_stack([t, np.zeros_like(t),
                                np.where(t < 5.0, -0.5, 0.0)])
    # The EKF turns right 721 deg by 5 s and stays there.
    yaw = -np.radians(721.0) * np.minimum(t / 5.0, 1.0)
    poses = np.column_stack([t, np.zeros_like(t), np.zeros_like(t),
                             np.remainder(yaw + np.pi, 2 * np.pi) - np.pi])
    turns, sides = overshoots(poses, commands, moves_of("spin -720"))
    assert turns == [pytest.approx(1.0)]
    assert sides == []


def test_a_side_past_its_length_counts_in_mm():
    # A 1 m side to 10 s, then a quarter turn left from 11.5 s to 14.5 s.
    t = np.arange(0.0, 20.0, 0.02)
    commands = np.column_stack([t, np.where(t < 10.0, 0.1, 0.0),
                                np.where((t >= 11.5) & (t < 14.5), 0.5, 0.0)])
    x = np.minimum(t, 10.02) * 0.1  # 1.002 m by the time it stops
    yaw = np.radians(90.0) * np.clip((t - 11.5) / 3.0, 0.0, 1.0)
    poses = np.column_stack([t, x, np.zeros_like(t), yaw])
    turns, sides = overshoots(poses, commands,
                              [("straight", 1.0), ("spin", 90.0)])
    assert sides == [pytest.approx(2.0, abs=0.01)]
    assert turns == [pytest.approx(0.0)]


def test_a_straight_run_has_a_side_and_no_turns():
    t = np.arange(0.0, 12.0, 0.02)
    commands = np.column_stack([t, np.where(t < 10.0, 0.1, 0.0),
                                np.zeros_like(t)])
    poses = np.column_stack([t, np.minimum(t, 10.0) * 0.1, np.zeros_like(t),
                             np.zeros_like(t)])
    turns, sides = overshoots(poses, commands, moves_of("straight 1.0"))
    assert turns == []
    assert sides == [pytest.approx(0.0, abs=0.01)]


def test_turns_that_dont_match_the_command_stop_the_tool():
    t = np.arange(0.0, 4.0, 0.02)
    commands = np.column_stack([t, np.zeros_like(t), np.full_like(t, 0.5)])
    poses = np.column_stack([t, np.zeros_like(t), np.zeros_like(t), t * 0.5])
    with pytest.raises(SystemExit):
        overshoots(poses, commands, moves_of("square 1.0"))


def test_the_pose_change_is_only_inside_the_window():
    poses = np.array([(0.0, 5.0, 5.0, 0.0),  # before the drive
                      (1.0, 0.0, 0.0, math.pi / 2),
                      (2.0, 0.0, 1.0, math.pi),
                      (3.0, 9.0, 9.0, 0.0)])  # after it
    assert pose_change(poses, 0.5, 2.5) == pytest.approx((1.0, 0.0, 90.0))


def test_a_window_without_poses_stops_the_tool():
    poses = np.array([(0.0, 0.0, 0.0, 0.0)])
    with pytest.raises(SystemExit):
        pose_change(poses, 5.0, 6.0)


def test_spread_is_the_sample_standard_deviation():
    assert sample_sd([1.0, 3.0]) == pytest.approx(math.sqrt(2.0))
    assert math.isnan(sample_sd([1.0]))
