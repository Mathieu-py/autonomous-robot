"""Compare where each estimate says the robot ended with where it really
ended, after odom_drive.py squares or spins measured on the floor, and show
how far each move ran past its target by our EKF.

Usage (ROS 2 sourced): python3 tools/end_pose_errors.py <bag dir> <floor csv>

Each drive is one odom_drive.py run: its /cmd_vel messages, with gaps of
under 5 s. The floor CSV names each measured drive by its index in the
recording, counting from 0, with the odom_drive command and how far each deck
flag ended from its start mark (results/README.md#square-tests-squares).
"""
import csv
import math
import sys
from collections import namedtuple

import numpy as np

from bag_reader import bag_messages, header_stamp_s
from odom_drive import PoseTracker, moves_for, yaw_from_quaternion
from runs import on_spans

EKF_TOPIC = "/ekf/odom"
ESTIMATES = {EKF_TOPIC: "our EKF",
             "/odometry/filtered": "robot_localization",
             "/odom": "wheels (/odom)"}
DRIVE_GAP_S = 5.0  # between odom_drive runs
BEFORE_S = 0.5  # before a drive's first command
AFTER_S = 0.3  # after its last, which holds zero while the robot settles
SETTLED_S = 1.3  # after a move's commands; the runs held zero 1.5 s between
FLAG_SPACING_MM = 240.0  # front to rear flag; the turning centre is midway

# Along the start heading; the turn counts whole turns.
PoseChange = namedtuple("PoseChange", "ahead_m left_m turn_deg")
# A positive turn error: the estimate counted more turn than the robot made.
EstimateError = namedtuple("EstimateError", "turn_error_deg end_error_mm")


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
        stamp_s = header_stamp_s(msg)
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


def overshoots(ekf_poses, commands, moves):
    """How far each turn and side of the moves ran past its target by our
    EKF: turns past the planned turn so far (deg), sides past their length
    (mm)."""
    t = commands[:, 0]
    turn_spans = on_spans(t, commands[:, 2] != 0)
    side_spans = on_spans(t, commands[:, 1] != 0)
    spin_degs = [amount for kind, amount in moves if kind == "spin"]
    side_m = max((abs(amount) for kind, amount in moves if kind == "straight"),
                 default=0.0)
    if len(turn_spans) != len(spin_degs):
        sys.exit(f"The drive has {len(turn_spans)} turns but its command "
                 f"makes {len(spin_degs)}: is its index in the floor CSV "
                 "right?")
    start_s = t[0] - BEFORE_S
    planned_so_far_deg = np.cumsum(spin_degs)
    turns = []
    for spin_deg, planned_deg, (_, end_s) in zip(spin_degs, planned_so_far_deg,
                                                 turn_spans):
        change = pose_change(ekf_poses, start_s, end_s + SETTLED_S)
        turned_deg = change.turn_deg
        turns.append((turned_deg - planned_deg) * math.copysign(1.0, spin_deg))
    sides = []
    for first_s, end_s in side_spans:
        change = pose_change(ekf_poses, first_s - BEFORE_S, end_s + SETTLED_S)
        side_mm = math.hypot(change.ahead_m, change.left_m) * 1000
        sides.append(side_mm - side_m * 1000)
    return turns, sides


def pose_change(poses, start_s, end_s):
    """The PoseChange from start_s to end_s."""
    tracker = PoseTracker()
    for _, x, y, yaw in poses[(poses[:, 0] >= start_s) & (poses[:, 0] <= end_s)]:
        tracker.feed(x, y, yaw)
    if tracker.start is None:
        sys.exit(f"No pose between {start_s:.1f} s and {end_s:.1f} s")
    ahead_m, left_m = tracker.offset_from_start()
    return PoseChange(ahead_m, left_m, math.degrees(tracker.turn_rad))


def planned_turn_deg(moves):
    """The turn the moves make: a left square is 360, 'spin -720' -720."""
    return sum(amount for kind, amount in moves if kind == "spin")


def moves_of(command):
    """An odom_drive command line's moves, such as 'square -1.0'."""
    kind, amount = command.split()
    return moves_for(kind, float(amount))


def floor_end_pose(planned_deg, front_along_mm, front_across_mm,
                   rear_along_mm, rear_across_mm):
    """The PoseChange from each flag's distance from its start mark (along +
    ahead, across + right); the turn is the planned whole turns plus what's
    left over."""
    left_over_deg = math.degrees(math.atan2(
        rear_across_mm - front_across_mm,
        FLAG_SPACING_MM + front_along_mm - rear_along_mm))
    return PoseChange((front_along_mm + rear_along_mm) / 2000,
                      -(front_across_mm + rear_across_mm) / 2000,
                      planned_deg + left_over_deg)


def errors(estimate, real, planned_deg):
    """The estimate's EstimateError against the real PoseChange."""
    turn_error_deg = ((estimate.turn_deg - real.turn_deg)
                      * math.copysign(1.0, planned_deg))
    end_error_mm = math.hypot(estimate.ahead_m - real.ahead_m,
                              estimate.left_m - real.left_m) * 1000
    return EstimateError(turn_error_deg, end_error_mm)


def sample_sd(values):
    """The spread of a few squares or turns; nan for fewer than two."""
    return float(np.std(values, ddof=1)) if len(values) > 1 else math.nan


def print_summary(title, rows):
    if not rows:
        return
    print(f"\n{title} ({len(rows)}):")
    real_mm = [math.hypot(r["real"].ahead_m, r["real"].left_m) * 1000
               for r in rows]
    heading_deg = [abs(r["real"].turn_deg - r["planned"]) for r in rows]
    print(f"  robot ended {np.mean(real_mm):.1f} mm and "
          f"{np.mean(heading_deg):.2f} deg from its start")
    for name in ESTIMATES.values():
        turns = [r[name].turn_error_deg for r in rows]
        ends = [r[name].end_error_mm for r in rows]
        per_turn = np.mean([t / abs(r["planned"]) for t, r in zip(turns, rows)])
        print(f"  {name:19s} turn {np.mean(turns):+.2f} deg "
              f"(spread {sample_sd(turns):.2f}, {100 * per_turn:+.3f}%), "
              f"end {np.mean(ends):4.1f} mm off (worst {max(ends):4.1f})")


def print_overshoots(turns, sides):
    parts = []
    if turns:
        parts.append("turns " + " ".join(f"{turn:+.2f}" for turn in turns)
                     + " deg")
    if sides:
        parts.append("sides " + " ".join(f"{side:+.1f}" for side in sides)
                     + " mm")
    print("  our EKF, past each target: " + "; ".join(parts))


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
        if not 0 <= drive < len(drive_commands):
            sys.exit(f"Drive {drive} isn't in the recording: it has "
                     f"{len(drive_commands)}")
        first_s, last_s = drive_commands[drive][[0, -1], 0]
        moves = moves_of(row["command"])
        planned = planned_turn_deg(moves)
        real = floor_end_pose(planned, *(float(row[column]) for column in (
            "front_along_mm", "front_across_mm", "rear_along_mm",
            "rear_across_mm")))
        result = {"command": row["command"], "planned": planned, "real": real}
        real_mm = math.hypot(real.ahead_m, real.left_m) * 1000
        print(f"{row['run']} (drive {drive}, {row['command']}): floor "
              f"{real.turn_deg:+.2f} deg, {real_mm:.1f} mm from the start")
        for topic, name in ESTIMATES.items():
            estimate = pose_change(poses[topic], first_s - BEFORE_S,
                                   last_s + AFTER_S)
            result[name] = errors(estimate, real, planned)
            error = result[name]
            print(f"  {name:19s} turn {error.turn_error_deg:+.2f} deg, end "
                  f"{error.end_error_mm:4.1f} mm off")
        turns, sides = overshoots(poses[EKF_TOPIC], drive_commands[drive],
                                  moves)
        print_overshoots(turns, sides)
        all_turns += turns
        all_sides += sides
        results.append(result)

    print_summary("Squares", [r for r in results
                              if r["command"].startswith("square")])
    print_summary("Spins", [r for r in results
                            if r["command"].startswith("spin")])
    parts = []
    if all_turns:
        parts.append(f"turns {np.mean(all_turns):+.2f} deg (spread "
                     f"{sample_sd(all_turns):.2f})")
    if all_sides:
        parts.append(f"sides {np.mean(all_sides):+.1f} mm (spread "
                     f"{sample_sd(all_sides):.1f})")
    print("\nPast each target by our EKF: " + ", ".join(parts))


if __name__ == "__main__":
    main()
