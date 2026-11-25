"""Plot a PID step-response CSV (or a PuTTY log containing the CSV lines).

Usage: python3 tools/plot_pid.py <input.csv> <output.png> "<title>"
"""
import sys

import matplotlib.pyplot as plt

from chart_style import BLUE, INK_2, ORANGE, plot_target, read_log_rows


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    log_path, out_path, title = sys.argv[1], sys.argv[2], sys.argv[3]

    rows = read_log_rows(log_path, 6)

    (t_ms, target, first_speed, first_power, second_speed,
     second_power) = zip(*rows)
    t = [ms / 1000 for ms in t_ms]

    fig, (ax_speed, ax_power) = plt.subplots(
        2, 1, figsize=(10, 6.5), sharex=True,
        gridspec_kw={"height_ratios": [3, 2], "hspace": 0.12})

    for ax in (ax_speed, ax_power):
        ax.axhline(0, color=INK_2, linewidth=0.8)

    plot_target(ax_speed, t, target, "Target")
    ax_speed.plot(t, first_speed, color=BLUE, linewidth=2, label="First wheel")
    ax_speed.plot(t, second_speed, color=ORANGE, linewidth=2,
                  label="Second wheel")
    ax_speed.set_ylabel("Speed (rev/s)")
    ax_speed.set_title(title, loc="left", fontsize=12, pad=10)
    ax_speed.legend(loc="upper right", ncol=3)

    ax_power.plot(t, first_power, color=BLUE, linewidth=2)
    ax_power.plot(t, second_power, color=ORANGE, linewidth=2)
    ax_power.set_ylabel("Motor power (PWM fraction)")
    ax_power.set_xlabel("Time since test start (s)")

    # Higher line's label on top, so the labels never overlap.
    upper, lower = (("First", first_power[-1]), ("Second", second_power[-1]))
    if lower[1] > upper[1]:
        upper, lower = lower, upper
    for (name, y), offset in ((upper, 6), (lower, -6)):
        ax_power.annotate(name, (t[-1], y), xytext=(6, offset),
                          textcoords="offset points", color=INK_2, va="center")

    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"{len(rows)} samples plotted to {out_path}")


if __name__ == "__main__":
    main()
