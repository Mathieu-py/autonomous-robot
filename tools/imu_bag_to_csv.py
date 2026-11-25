"""Export /imu/data_raw from a ROS 2 bag to CSV.

Usage: python3 tools/imu_bag_to_csv.py <bag folder> <output.csv>
Columns: t_s (from the first message), accel x y z (m/s^2), gyro x y z (rad/s),
in the IMU's own axes, gyro offset already removed by the bridge.
"""
import csv
import sys

from bag_reader import bag_messages


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    bag_path, out_path = sys.argv[1], sys.argv[2]

    rows = []
    for _, imu in bag_messages(bag_path, ["/imu/data_raw"]):
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
            writer.writerow([f"{row[0] - start:.4f}"]
                            + [f"{v:.5f}" for v in row[1:]])
    duration_s = rows[-1][0] - start
    print(f"{len(rows)} samples over {duration_s:.1f} s -> {out_path}")


if __name__ == "__main__":
    main()
