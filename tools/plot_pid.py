"""Plot a PID step-response CSV (or a PuTTY log containing the CSV lines).

Usage: python3 tools/plot_pid.py <input.csv> <output.png> "<title>"
"""
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

if len(sys.argv) != 4:
    sys.exit(__doc__)
log_path, out_path, title = sys.argv[1], sys.argv[2], sys.argv[3]

rows = []
with open(log_path, encoding="utf-8", errors="replace") as f:
    for line in f:
        parts = line.strip().split(",")
        if len(parts) != 6:
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:
            continue

if not rows:
    sys.exit(f"No PID log lines found in {log_path}")

t = [r[0] / 1000 for r in rows]
target = [r[1] for r in rows]
first_speed = [r[2] for r in rows]
first_power = [r[3] for r in rows]
second_speed = [r[4] for r in rows]
second_power = [r[5] for r in rows]

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
BLUE = "#2a78d6"
ORANGE = "#eb6834"

plt.rcParams.update({
    "font.size": 10,
    "axes.edgecolor": GRID,
    "axes.labelcolor": INK_2,
    "xtick.color": INK_2,
    "ytick.color": INK_2,
    "axes.titlecolor": INK,
})

fig, (ax_speed, ax_power) = plt.subplots(
    2, 1, figsize=(10, 6.5), sharex=True, facecolor=SURFACE,
    gridspec_kw={"height_ratios": [3, 2], "hspace": 0.12})

for ax in (ax_speed, ax_power):
    ax.set_facecolor(SURFACE)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.spines[["top", "right"]].set_visible(False)
    ax.axhline(0, color=INK_2, linewidth=0.8)

ax_speed.step(t, target, where="post", color=INK_2, linewidth=1.5,
              linestyle=(0, (4, 3)), label="Target")
ax_speed.plot(t, first_speed, color=BLUE, linewidth=2, label="First wheel")
ax_speed.plot(t, second_speed, color=ORANGE, linewidth=2, label="Second wheel")
ax_speed.set_ylabel("Speed (rev/s)")
ax_speed.set_title(title, loc="left", fontsize=12, pad=10)
ax_speed.legend(loc="upper right", frameon=False, ncol=3)

ax_power.plot(t, first_power, color=BLUE, linewidth=2)
ax_power.plot(t, second_power, color=ORANGE, linewidth=2)
ax_power.set_ylabel("Motor power (PWM fraction)")
ax_power.set_xlabel("Time since test start (s)")

# Direct labels at the end of each power line, the higher line's label on
# top so they never swap or overlap.
upper, lower = (("First", first_power[-1]), ("Second", second_power[-1]))
if lower[1] > upper[1]:
    upper, lower = lower, upper
for (name, y), offset in ((upper, 6), (lower, -6)):
    ax_power.annotate(name, (t[-1], y), xytext=(6, offset),
                      textcoords="offset points", color=INK_2, va="center")

fig.savefig(out_path, dpi=150, bbox_inches="tight", facecolor=SURFACE)
print(f"{len(rows)} samples plotted to {out_path}")
