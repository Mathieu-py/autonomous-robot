import numpy as np
import pytest

from odom_imu_noise import SETTLE_S, STOPPING_S, about_run_means, stretches


def test_finds_each_unbroken_stretch():
    t = np.arange(8.0)
    is_on = np.array([0, 1, 1, 0, 0, 1, 1, 1], dtype=bool)
    assert stretches(t, is_on) == [(1.0, 3.0), (5.0, 7.0)]


def test_measures_each_run_about_its_own_mean():
    t = np.arange(0.0, 10.0, 0.02)
    forward = (t > 1) & (t < 4)
    backward = (t > 6) & (t < 9)
    values = np.where(forward, 0.1, 0.0) + np.where(backward, -0.1, 0.0)
    spread = about_run_means(t, values, [(1.0, 4.0), (6.0, 9.0)])
    # Opposite runs don't count as spread.
    assert spread.std() == pytest.approx(0.0)


def test_drops_speeding_up_and_slowing_down():
    t = np.arange(0.0, 10.0, 0.01)
    values = np.where((t > 2 + SETTLE_S) & (t < 8 - STOPPING_S), 1.0, 0.0)
    assert about_run_means(t, values, [(2.0, 8.0)]).std() == \
        pytest.approx(0.0)
