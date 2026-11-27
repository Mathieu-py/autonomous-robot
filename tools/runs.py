"""Splits a series of readings into runs of one value."""
from itertools import groupby


def spans(t, values):
    """(start, end, value) for each run of one value; it ends at the next
    reading, or the last one."""
    result = []
    start = 0
    for value, run in groupby(values):
        end = start + len(list(run))
        result.append((t[start], t[min(end, len(t) - 1)], value))
        start = end
    return result


def on_spans(t, is_on):
    """(start, end) of each run of True."""
    return [(start, end) for start, end, on in spans(t, is_on) if on]
