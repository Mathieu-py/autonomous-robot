import numpy as np

from runs import on_spans, spans


def test_each_run_lasts_until_the_next_reading():
    t = [0.0, 0.02, 0.04, 0.06]
    assert spans(t, ["none", "bumper", "bumper", "none"]) == [
        (0.0, 0.02, "none"), (0.02, 0.06, "bumper"), (0.06, 0.06, "none")]


def test_back_to_back_runs_stay_separate():
    t = [0.0, 0.02, 0.04, 0.06]
    assert spans(t, ["bumper", "bumper", "low battery", "none"])[:2] == [
        (0.0, 0.04, "bumper"), (0.04, 0.06, "low battery")]


def test_a_run_still_on_at_the_end_lasts_to_the_last_reading():
    t = [0.0, 0.02, 0.04]
    assert spans(t, ["none", "low battery", "low battery"])[-1] == \
        (0.02, 0.04, "low battery")


def test_finds_each_run_of_true():
    t = np.arange(8.0)
    is_on = np.array([0, 1, 1, 0, 0, 1, 1, 1], dtype=bool)
    assert on_spans(t, is_on) == [(1.0, 3.0), (5.0, 7.0)]
