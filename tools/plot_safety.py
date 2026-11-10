"""Plot a safety-test CSV: wheel speeds, motor power and battery voltage, with
each stop shaded and labelled.

Usage: python3 tools/plot_safety.py <input.csv> <output.png> "<title>"
"""
import sys

import matplotlib.pyplot as plt

from chart_style import (BLUE, INK, INK_2, ORANGE, STOP_FILL, SURFACE,
                         read_log_rows, style_axes)

if len(sys.argv) != 4:
    sys.exit(__doc__)
log_path, out_path, title = sys.argv[1], sys.argv[2], sys.argv[3]

rows = read_log_rows(log_path, 8, text_columns=1)
if not rows:
    sys.exit(f"No safety log lines found in {log_path}")

t = [r[0] / 1000 for r in rows]
setpoint = [r[1] for r in rows]
first_speed, first_power = [r[2] for r in rows], [r[3] for r in rows]
second_speed, second_power = [r[4] for r in rows], [r[5] for r in rows]
battery = [r[6] for r in rows]
stops = [r[7] for r in rows]

fig, (ax_speed, ax_power, ax_battery) = plt.subplots(
    3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE,
    gridspec_kw={"height_ratios": [3, 2, 2], "hspace": 0.12})

# Shade each stop and label it with its reason.
stop_spans = []
start = None
for i, reason in enumerate(stops):
    if reason != "none" and start is None:
        start = i
    if start is not None and (reason == "none" or i == len(stops) - 1):
        stop_spans.append((t[start], t[i], stops[start]))
        start = None
for ax in (ax_speed, ax_power, ax_battery):
    style_axes(ax)
    for begin, end, _ in stop_spans:
        ax.axvspan(begin, end, color=STOP_FILL, linewidth=0, zorder=0)
for begin, _, reason in stop_spans:
    ax_speed.annotate(reason, (begin, 1.0), xycoords=("data", "axes fraction"),
                      xytext=(3, -3), textcoords="offset points",
                      color=INK_2, fontsize=8, va="top", rotation=90)

ax_speed.step(t, setpoint, where="post", color=INK_2, linewidth=1.5,
              linestyle=(0, (4, 3)), label="Setpoint")
ax_speed.plot(t, first_speed, color=BLUE, linewidth=1.5, label="First wheel")
ax_speed.plot(t, second_speed, color=ORANGE, linewidth=1.5,
              label="Second wheel")
ax_speed.set_ylabel("Speed (rev/s)")
ax_speed.set_title(title, loc="left", fontsize=12, pad=10)
ax_speed.legend(loc="lower right", frameon=False, ncol=3)

ax_power.plot(t, first_power, color=BLUE, linewidth=1.5)
ax_power.plot(t, second_power, color=ORANGE, linewidth=1.5)
ax_power.set_ylabel("Motor power")

ax_battery.plot(t, battery, color=INK, linewidth=1.5)
ax_battery.axhline(10.5, color=INK_2, linewidth=1, linestyle=(0, (2, 2)))
ax_battery.annotate("10.5 V cutoff", (t[-1], 10.5), xytext=(0, 3),
                    textcoords="offset points", color=INK_2, ha="right",
                    va="bottom")
ax_battery.set_ylabel("Battery (V, filtered)")
ax_battery.set_xlabel("Time since test start (s); shaded = motors stopped")

fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
print(f"{len(rows)} samples, {len(stop_spans)} stops, plotted to {out_path}")
