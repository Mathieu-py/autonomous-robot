import numpy as np
import pytest

from odom_imu_noise import (SPEEDING_UP_S, STOPPING_S, about_run_means,
                            long_runs)


def test_keeps_only_runs_longer_than_a_square_corner():
    t = np.arange(0.0, 20.0, 0.1)
    is_on = (t > 1) & (t < 4.5) | (t > 8) & (t < 18)  # a corner, then a run
    assert long_runs(t, is_on) == [(pytest.approx(8.1), pytest.approx(18.0))]


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
    values = np.where((t > 2 + SPEEDING_UP_S) & (t < 8 - STOPPING_S), 1.0,
                      0.0)
    assert about_run_means(t, values, [(2.0, 8.0)]).std() == \
        pytest.approx(0.0)
