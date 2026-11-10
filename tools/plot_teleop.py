"""Turn a teleop rosbag into a CSV and a chart: commanded vs measured wheel
speeds, battery voltage, and the bridge node's log messages.

Usage (with ROS 2 sourced):
  python3 tools/plot_teleop.py <bag dir> <output.csv> <output.png> "<title>"

Reads messages in the order they were written, and times them by counting
/joint_states (the Pico reports exactly every 20 ms): WSL's clock steps back
about a second every 30 s, which scrambles anything sorted by timestamp.
Commanded wheel speeds are rebuilt from /cmd_vel the way pico_bridge does it:
the last twist holds for cmd_vel_timeout_s, then drops to zero.
"""
import csv
import math
import sys

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from chart_style import BLUE, INK, INK_2, ORANGE, SURFACE, style_axes

WHEEL_DIAMETER_M = 0.068  # pico_bridge defaults
WHEEL_SEPARATION_M = 0.20
CMD_VEL_TIMEOUT_S = 1.0
STATUS_PERIOD_S = 0.02
RADIANS_PER_REV = 2 * math.pi

if len(sys.argv) != 5:
    sys.exit(__doc__)
bag_path, csv_path, png_path, title = sys.argv[1:]

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
reader.set_read_order(rosbag2_py.ReadOrder(sort_by=rosbag2_py.ReadOrderSortBy.File))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}

metres_per_rev = math.pi * WHEEL_DIAMETER_M
half_track = WHEEL_SEPARATION_M / 2
rows, logs = [], []
twist, twist_time = (0.0, 0.0), -math.inf
battery = math.nan
while reader.has_next():
    topic, data, _ = reader.read_next()
    msg = deserialize_message(data, get_message(types[topic]))
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

t = [r[0] for r in rows]
fig, (ax_left, ax_right, ax_battery) = plt.subplots(
    3, 1, figsize=(11, 8), sharex=True, facecolor=SURFACE,
    gridspec_kw={"height_ratios": [2, 2, 1.3], "hspace": 0.15})
for ax, name, cmd, measured, color in (
        (ax_left, "Left wheel", 1, 2, BLUE),
        (ax_right, "Right wheel", 3, 4, ORANGE)):
    ax.step(t, [r[cmd] for r in rows], where="post", color=INK_2,
            linewidth=1.5, linestyle=(0, (4, 3)), label="Commanded")
    ax.plot(t, [r[measured] for r in rows], color=color, linewidth=1.5,
            label="Measured")
    ax.set_ylabel(f"{name} (rev/s)")
    ax.legend(loc="upper right", frameon=False, ncol=2)
ax_battery.plot(t, [r[5] for r in rows], color=INK, linewidth=1.5)
ax_battery.set_ylabel("Battery (V)")
ax_battery.set_xlabel("Time from the first wheel reading (s)")
for ax in (ax_left, ax_right, ax_battery):
    style_axes(ax)
    for log_t, _ in logs:
        ax.axvline(log_t, color=INK_2, linewidth=0.8, linestyle=(0, (1, 2)))
for log_t, text in logs:
    ax_left.annotate(text, (log_t, 1.0), xycoords=("data", "axes fraction"),
                     xytext=(3, -3), textcoords="offset points", color=INK_2,
                     fontsize=7, va="top", rotation=90)
ax_left.set_title(title, loc="left", fontsize=12, pad=10)
fig.savefig(png_path, dpi=150, bbox_inches="tight", facecolor=SURFACE)

print(f"{len(rows)} samples, {len(logs)} bridge log lines")
for log_t, text in logs:
    print(f"  {log_t:7.2f} s  {text}")
