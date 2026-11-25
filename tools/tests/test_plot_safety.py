from plot_safety import stop_spans


def test_each_stop_runs_until_the_next_reading():
    t = [0.0, 0.02, 0.04, 0.06]
    assert stop_spans(t, ["none", "bumper", "bumper", "none"]) == [
        (0.02, 0.06, "bumper")]


def test_back_to_back_stops_stay_separate():
    t = [0.0, 0.02, 0.04, 0.06]
    assert stop_spans(t, ["bumper", "bumper", "low battery", "none"]) == [
        (0.0, 0.04, "bumper"), (0.04, 0.06, "low battery")]


def test_a_stop_still_on_at_the_end_runs_to_the_last_reading():
    t = [0.0, 0.02, 0.04]
    assert stop_spans(t, ["none", "low battery", "low battery"]) == [
        (0.02, 0.04, "low battery")]
