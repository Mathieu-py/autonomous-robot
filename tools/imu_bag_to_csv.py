"""Export /imu/data_raw from a ROS 2 bag to CSV.

Usage: python3 tools/imu_bag_to_csv.py <bag folder> <output.csv>
Columns: t_s (from the first message), accel x y z (m/s^2), gyro x y z (rad/s),
in the IMU's own axes, gyro offset already removed by the bridge.
"""
import csv
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

if len(sys.argv) != 3:
    sys.exit(__doc__)
bag_path, out_path = sys.argv[1], sys.argv[2]

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag_path),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
reader.set_filter(rosbag2_py.StorageFilter(topics=["/imu/data_raw"]))

rows = []
while reader.has_next():
    _, data, _ = reader.read_next()
    imu = deserialize_message(data, Imu)
    stamp = imu.header.stamp.sec + imu.header.stamp.nanosec * 1e-9
    a, g = imu.linear_acceleration, imu.angular_velocity
    rows.append([stamp, a.x, a.y, a.z, g.x, g.y, g.z])
if not rows:
    sys.exit(f"No /imu/data_raw messages in {bag_path}")

start = rows[0][0]
with open(out_path, "w", newline="") as out:
    writer = csv.writer(out)
    writer.writerow(["t_s", "ax", "ay", "az", "gx", "gy", "gz"])
    for row in rows:
        writer.writerow([f"{row[0] - start:.4f}"] + [f"{v:.5f}" for v in row[1:]])
print(f"{len(rows)} samples over {rows[-1][0] - start:.1f} s -> {out_path}")
