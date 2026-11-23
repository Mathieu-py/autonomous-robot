"""Turn a teleop rosbag into a CSV and a chart: commanded vs measured wheel
speeds, battery voltage, and the bridge node's log messages.

Usage (with ROS 2 sourced):
  python3 tools/plot_teleop.py <bag dir> <output.csv> <output.png> "<title>"
      [wheel separation in m]

The drive geometry comes from ros/robot_description/config/robot.yaml. Runs
01-04 (2026-11-09) were driven with a 0.20 m separation: pass 0.20 for those.

Times messages by counting /joint_states (every 20 ms), not by timestamp: WSL's
clock steps back about a second every 30 s.
"""
import csv
import math
import pathlib
import sys

import matplotlib.pyplot as plt
import yaml

from bag_reader import bag_messages
from chart_style import (BLUE, INK, INK_2, ORANGE, SURFACE, label_events,
                         plot_target, style_axes)

ROBOT_YAML = (pathlib.Path(__file__).resolve().parent.parent
              / "ros/robot_description/config/robot.yaml")
CMD_VEL_TIMEOUT_S = 1.0  # pico_bridge's cmd_vel_timeout_s
STATUS_PERIOD_S = 0.02
RADIANS_PER_REV = 2 * math.pi

if len(sys.argv) not in (5, 6):
    sys.exit(__doc__)
bag_path, csv_path, png_path, title = sys.argv[1:5]

drive = yaml.safe_load(ROBOT_YAML.read_text())["/**"]["ros__parameters"]
wheel_separation_m = (float(sys.argv[5]) if len(sys.argv) == 6
                      else drive["wheel_separation_m"])
metres_per_rev = math.pi * drive["wheel_diameter_m"]
half_track = wheel_separation_m / 2
rows, logs = [], []
twist, twist_time = (0.0, 0.0), -math.inf
battery = math.nan
for topic, msg in bag_messages(bag_path):
    t = len(rows) * STATUS_PERIOD_S
    if topic == "/cmd_vel":
        twist, twist_time = (msg.linear.x, msg.angular.z), t
    elif topic == "/battery":
        battery = msg.voltage
    elif topic == "/rosout" and msg.name == "pico_bridge":
        logs.append((t, msg.msg))
    elif topic == "/joint_states":
        linear, angular = twist if t - twist_time <= CMD_VEL_TIMEOUT_S else (0, 0)
        rows.append((t,
                     (linear - angular * half_track) / metres_per_rev,
                     msg.velocity[0] / RADIANS_PER_REV,
                     (linear + angular * half_track) / metres_per_rev,
                     msg.velocity[1] / RADIANS_PER_REV,
                     battery))
if not rows:
    sys.exit(f"No /joint_states in {bag_path}")

with open(csv_path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["t_ms", "left_command_rps", "left_rps",
                     "right_command_rps", "right_rps", "battery_volts"])
    for r in rows:
        writer.writerow([round(r[0] * 1000), f"{r[1]:.3f}", f"{r[2]:.3f}",
                         f"{r[3]:.3f}", f"{r[4]:.3f}", f"{r[5]:.2f}"])

t, left_cmd, left, right_cmd, right, battery_v = zip(*rows)
fig, (ax_left, ax_right, ax_battery) = plt.subplots(
    3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE,
    gridspec_kw={"height_ratios": [2, 2, 1.3], "hspace": 0.15})
for ax, name, commanded, measured, color in (
        (ax_left, "Left wheel", left_cmd, left, BLUE),
        (ax_right, "Right wheel", right_cmd, right, ORANGE)):
    plot_target(ax, t, commanded, "Commanded")
    ax.plot(t, measured, color=color, linewidth=1.5, label="Measured")
    ax.set_ylabel(f"{name} (rev/s)")
    ax.legend(loc="upper right", frameon=False, ncol=2)
ax_battery.plot(t, battery_v, color=INK, linewidth=1.5)
ax_battery.set_ylabel("Battery (V)")
ax_battery.set_xlabel("Time from the first wheel reading (s)")
for ax in (ax_left, ax_right, ax_battery):
    style_axes(ax)
    for log_t, _ in logs:
        ax.axvline(log_t, color=INK_2, linewidth=0.8, linestyle=(0, (1, 2)))
label_events(ax_left, logs, fontsize=7)
ax_left.set_title(title, loc="left", fontsize=12, pad=10)
fig.savefig(png_path, dpi=150, bbox_inches="tight", facecolor=SURFACE)

print(f"{len(rows)} samples, {len(logs)} bridge log lines")
for log_t, text in logs:
    print(f"  {log_t:7.2f} s  {text}")
