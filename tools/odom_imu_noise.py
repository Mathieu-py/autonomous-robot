"""Measure the noise in /odom and /imu/data_raw from a recording of
odom_drive.py runs, for their covariance parameters.

Usage (ROS 2 sourced): python3 tools/odom_imu_noise.py <bag dir>

Uses the first 30 s as still, finds the straight runs and the spins from
/odom's own speeds, drops each run's speeding up and slowing down, and
reports each reading's spread about its run's average. Also compares each
spin's angle by gyro with /odom's.
"""
import sys

import numpy as np

from bag_reader import bag_messages

MOVING_MPS = 0.05
TURNING_RADPS = 0.25
STILL_S = 30.0  # untouched at the start of the recording
SETTLE_S = 1.0  # speeding up at the start of each run
STOPPING_S = 0.5  # slowing down at the end of each run
MIN_RUN_S = 3.0
COAST_S = 2.0  # after a spin, still turning a little


def stretches(t, is_on):
    """(start, end) times of each unbroken run of True in is_on."""
    result = []
    start = None
    for time, on in zip(t, is_on):
        if on and start is None:
            start = time
        elif not on and start is not None:
            result.append((start, time))
            start = None
    if start is not None:
        result.append((start, t[-1]))
    return result


def about_run_means(t, values, runs):
    """Each run's steady values, about that run's mean."""
    parts = [values[(t > start + SETTLE_S) & (t < end - STOPPING_S)]
             for start, end in runs]
    return np.concatenate([part - part.mean() for part in parts])


def load(bag_path):
    odom, imu = [], []
    for topic, msg in bag_messages(bag_path, ["/odom", "/imu/data_raw"]):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == "/odom":
            q = msg.pose.pose.orientation
            odom.append((stamp, msg.twist.twist.linear.x,
                         msg.twist.twist.angular.z, 2 * np.arctan2(q.z, q.w)))
        else:
            imu.append((stamp, msg.angular_velocity.z))
    if not odom or not imu:
        sys.exit(f"No /odom or /imu/data_raw in {bag_path}")
    return np.array(odom), np.array(imu)


def report(label, values, unit):
    print(f"  {label:24s} spread {values.std():.2e} {unit}, "
          f"variance {values.var():.1e}")


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    odom, imu = load(sys.argv[1])
    t0 = odom[0, 0]
    t, speed_mps, turn_radps = odom[:, 0] - t0, odom[:, 1], odom[:, 2]
    yaw_rad = np.unwrap(odom[:, 3])
    imu_t, gyro_z = imu[:, 0] - t0, imu[:, 1]
    gyro_at_odom = np.interp(t, imu_t, gyro_z)

    is_straight = np.abs(speed_mps) > MOVING_MPS
    is_spin = np.abs(turn_radps) > TURNING_RADPS
    straights = [r for r in stretches(t, is_straight)
                 if r[1] - r[0] > MIN_RUN_S]
    spins = [r for r in stretches(t, is_spin) if r[1] - r[0] > MIN_RUN_S]
    is_still = imu_t < STILL_S

    print(f"Still for {STILL_S:.0f} s, {len(straights)} straight runs, "
          f"{len(spins)} spins")
    print("Still:")
    report("gyro z", gyro_z[is_still], "rad/s")
    for name, runs in (("Straight runs", straights), ("Spins", spins)):
        print(f"{name}:")
        report("odom speed", about_run_means(t, speed_mps, runs), "m/s")
        report("odom turn rate", about_run_means(t, turn_radps, runs),
               "rad/s")
        report("gyro z", about_run_means(t, gyro_at_odom, runs), "rad/s")
        report("odom turn - gyro z",
               about_run_means(t, turn_radps - gyro_at_odom, runs), "rad/s")

    print("Spin angles, gyro against /odom:")
    for start, end in spins:
        window = (t > start - SETTLE_S) & (t < end + COAST_S)
        imu_window = (imu_t > start - SETTLE_S) & (imu_t < end + COAST_S)
        odom_deg = np.degrees(yaw_rad[window][-1] - yaw_rad[window][0])
        gyro_deg = np.degrees(np.trapz(gyro_z[imu_window],
                                       imu_t[imu_window]))
        print(f"  /odom {odom_deg:+8.2f} deg, gyro {gyro_deg:+8.2f} deg, "
              f"gyro / odom {gyro_deg / odom_deg:.4f}")


if __name__ == "__main__":
    main()
