"""Reads the firmware's CSV rows from serial logs."""
import sys


def read_log_rows(path, column_count, text_columns=0):
    """The firmware's CSV rows, also from a PuTTY log with other lines mixed in.
    The last text_columns fields stay text; the rest are parsed as floats.
    Exits if there are none."""
    rows = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) != column_count:
                continue
            numbers = parts[:column_count - text_columns]
            try:
                rows.append([float(p) for p in numbers]
                            + parts[len(numbers):])
            except ValueError:
                continue
    if not rows:
        sys.exit(f"No {column_count}-column log lines found in {path}")
    return rows
