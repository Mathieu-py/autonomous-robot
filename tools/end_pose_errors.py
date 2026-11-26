"""Compare where each estimate says the robot ended with where it really
ended, after odom_drive.py squares or spins measured on the floor, and show
how far each move ran past its target by our EKF.

Usage (ROS 2 sourced): python3 tools/end_pose_errors.py <bag dir> <floor csv>

Each drive is one odom_drive.py run: its /cmd_vel messages, with gaps of
under 5 s. The floor CSV names each measured drive by its index in the
recording, counting from 0, with the odom_drive command and how far each deck
flag ended from its start mark (results/README.md#square-tests).
"""
import csv
import math
import sys

import numpy as np

from bag_reader import bag_messages
from odom_drive import PoseTracker, yaw_from_quaternion
from odom_imu_noise import stretches

EKF_TOPIC = "/ekf/odom"
ESTIMATES = {EKF_TOPIC: "our EKF",
             "/odometry/filtered": "robot_localization",
             "/odom": "wheels (/odom)"}
DRIVE_GAP_S = 5.0  # between odom_drive runs
BEFORE_S = 0.5  # before a drive's first command
AFTER_S = 0.3  # after its last, which holds zero while the robot settles
SETTLED_S = 1.3  # after a move's commands end; the next starts 1.5 s later
FLAG_SPACING_MM = 240.0  # front to rear flag; the turning centre is midway


def load(bag_path):
    """Each estimate's (stamp, x, y, yaw) and each /cmd_vel as (time, speed,
    turn rate), timed by the robot's clock: the stamp of the /odom recorded
    just before it, since a recording made on the laptop has the laptop's
    clock."""
    poses = {topic: [] for topic in ESTIMATES}
    commands = []
    latest_odom_s = None
    for topic, msg in bag_messages(bag_path, [*ESTIMATES, "/cmd_vel"]):
        if topic == "/cmd_vel":
            if latest_odom_s is not None:
                commands.append((latest_odom_s, msg.linear.x, msg.angular.z))
            continue
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position, rotation = msg.pose.pose.position, msg.pose.pose.orientation
        poses[topic].append((stamp_s, position.x, position.y,
                             yaw_from_quaternion(rotation.z, rotation.w)))
        if topic == "/odom":
            latest_odom_s = stamp_s
    if not commands or not all(poses.values()):
        sys.exit(f"No /cmd_vel, or a pose topic missing, in {bag_path}")
    return {topic: np.array(rows) for topic, rows in poses.items()}, \
        np.array(commands)


def drives(commands):
    """Each drive's commands."""
    breaks = np.flatnonzero(np.diff(commands[:, 0]) > DRIVE_GAP_S) + 1
    return np.split(commands, breaks)


def overshoots(ekf_poses, commands, planned_deg, side_m):
    """How far each turn and side ran past its target by our EKF: turns past
    the planned turn so far (deg), sides past side_m (mm)."""
    t = commands[:, 0]
    turn_spans = stretches(t, commands[:, 2] != 0)
    side_spans = stretches(t, commands[:, 1] != 0)
    start_s = t[0] - BEFORE_S
    turn_step_deg = abs(planned_deg) / len(turn_spans)
    turns = [math.copysign(1.0, planned_deg)
             * pose_change(ekf_poses, start_s, end_s + SETTLED_S)[2]
             - turn_step_deg * (k + 1)
             for k, (_, end_s) in enumerate(turn_spans)]
    sides = [(math.hypot(*pose_change(ekf_poses, first_s - BEFORE_S,
                                      end_s + SETTLED_S)[:2]) - side_m) * 1000
             for first_s, end_s in side_spans]
    return turns, sides


def pose_change(poses, start_s, end_s):
    """(ahead m, left m, turn deg) from start_s to end_s, along the start
    heading; the turn counts whole turns."""
    tracker = PoseTracker()
    for _, x, y, yaw in poses[(poses[:, 0] >= start_s) & (poses[:, 0] <= end_s)]:
        tracker.feed(x, y, yaw)
    ahead_m, left_m = tracker.offset_from_start()
    return ahead_m, left_m, math.degrees(tracker.turn_rad)


def planned_turn_deg(command):
    """The turn an odom_drive command makes: 'square -1.0' is one turn
    right, 'spin 720' two left."""
    kind, amount = command.split()
    if kind == "square":
        return math.copysign(360.0, float(amount))
    return float(amount)


def side_length_m(command):
    """A square's side; 0 for a spin."""
    kind, amount = command.split()
    return abs(float(amount)) if kind == "square" else 0.0


def floor_end_pose(planned_deg, front_along_mm, front_across_mm,
                   rear_along_mm, rear_across_mm):
    """(ahead m, left m, turn deg) from each flag's distance from its start
    mark (along + ahead, across + right); the turn is the planned whole turns
    plus what's left over."""
    left_over_deg = math.degrees(math.atan2(
        rear_across_mm - front_across_mm,
        FLAG_SPACING_MM + front_along_mm - rear_along_mm))
    return ((front_along_mm + rear_along_mm) / 2000,
            -(front_across_mm + rear_across_mm) / 2000,
            planned_deg + left_over_deg)


def errors(estimate, real, planned_deg):
    """(turn error deg, positive when it counted more turn than the robot
    made; end error mm)."""
    turn_error_deg = (estimate[2] - real[2]) * math.copysign(1.0, planned_deg)
    end_error_mm = math.hypot(estimate[0] - real[0], estimate[1] - real[1]) * 1000
    return turn_error_deg, end_error_mm


def print_summary(title, rows):
    if not rows:
        return
    print(f"\n{title} ({len(rows)}):")
    real_mm = [math.hypot(r["real"][0], r["real"][1]) * 1000 for r in rows]
    heading_deg = [abs(r["real"][2] - r["planned"]) for r in rows]
    print(f"  robot ended {np.mean(real_mm):.1f} mm and "
          f"{np.mean(heading_deg):.2f} deg from its start")
    for name in ESTIMATES.values():
        turns = [r[name][0] for r in rows]
        ends = [r[name][1] for r in rows]
        per_turn = np.mean([t / abs(r["planned"]) for t, r in zip(turns, rows)])
        print(f"  {name:19s} turn {np.mean(turns):+.2f} deg "
              f"(spread {np.std(turns):.2f}, {100 * per_turn:+.3f}%), "
              f"end {np.mean(ends):4.1f} mm off (worst {max(ends):4.1f})")


def print_overshoots(turns, sides):
    print("  our EKF, past each target: turns "
          + " ".join(f"{turn:+.2f}" for turn in turns) + " deg"
          + ("; sides " + " ".join(f"{side:+.1f}" for side in sides) + " mm"
             if sides else ""))


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    poses, commands = load(sys.argv[1])
    drive_commands = drives(commands)
    with open(sys.argv[2], newline="") as floor_file:
        floor_rows = list(csv.DictReader(floor_file))

    results = []
    all_turns, all_sides = [], []
    for row in floor_rows:
        drive = int(row["drive"])
        if drive >= len(drive_commands):
            sys.exit(f"Drive {drive} isn't in the recording: it has "
                     f"{len(drive_commands)}")
        first_s, last_s = drive_commands[drive][[0, -1], 0]
        planned = planned_turn_deg(row["command"])
        real = floor_end_pose(planned, *(float(row[column]) for column in (
            "front_along_mm", "front_across_mm", "rear_along_mm",
            "rear_across_mm")))
        result = {"command": row["command"], "planned": planned, "real": real}
        print(f"{row['run']} (drive {drive}, {row['command']}): floor "
              f"{real[2]:+.2f} deg, {math.hypot(real[0], real[1]) * 1000:.1f} "
              "mm from the start")
        for topic, name in ESTIMATES.items():
            estimate = pose_change(poses[topic], first_s - BEFORE_S,
                                   last_s + AFTER_S)
            result[name] = errors(estimate, real, planned)
            print(f"  {name:19s} turn {result[name][0]:+.2f} deg, end "
                  f"{result[name][1]:4.1f} mm off")
        turns, sides = overshoots(poses[EKF_TOPIC], drive_commands[drive],
                                  planned, side_length_m(row["command"]))
        print_overshoots(turns, sides)
        all_turns += turns
        all_sides += sides
        results.append(result)

    print_summary("Squares", [r for r in results
                              if r["command"].startswith("square")])
    print_summary("Spins", [r for r in results
                            if r["command"].startswith("spin")])
    print(f"\nPast each target by our EKF: turns {np.mean(all_turns):+.2f} "
          f"deg (spread {np.std(all_turns):.2f})"
          + (f", sides {np.mean(all_sides):+.1f} mm (spread "
             f"{np.std(all_sides):.1f})" if all_sides else ""))


if __name__ == "__main__":
    main()
