"""Drive a set distance, angle or square by the robot's pose (the odom ->
base_link transform, from our EKF), stop, and print where the robot ended by
that pose and by /odom alone, to compare with a tape measure.

Run it on the Pi: over WiFi the pose arrives late, so the stops vary.

Usage (ROS 2 sourced; the bridge and robot.launch.py running):
  python3 tools/odom_drive.py straight <metres>   # 0.1 m/s; negative reverses
  python3 tools/odom_drive.py spin <degrees>      # on the spot at 0.5 rad/s;
                                                  # negative spins right
  python3 tools/odom_drive.py square <metres>     # four sides, each then a
                                                  # 90 deg turn; negative turns
                                                  # right
"""
import math
import signal
import sys
import time

LINEAR_MPS = 0.1
ANGULAR_RADPS = 0.5
# As Nav2's velocity smoother defaults. The firmware's own wheel ramp allows
# about 1 m/s² straight but 12 rad/s² turning: a jolt at each turn.
LINEAR_ACCEL_MPS2 = 2.5
ANGULAR_ACCEL_RADPS2 = 3.2
MIN_SPEED_FRACTION = 0.2  # while closing in, so a move can't stall short
# How far the robot carries on after deciding to stop, by the EKF, with this
# script on the Pi (square tests, 26 Nov 2026). Each move aims short by this,
# as ROS By Example's angular_tolerance does for turns.
STRAIGHT_RUN_ON_M = 0.014
TURN_RUN_ON_RAD = math.radians(3.0)
# Per kind of move: cruise speed, acceleration, and how far it carries on.
MOVE_LIMITS = {"straight": (LINEAR_MPS, LINEAR_ACCEL_MPS2, STRAIGHT_RUN_ON_M),
               "spin": (ANGULAR_RADPS, ANGULAR_ACCEL_RADPS2, TURN_RUN_ON_RAD)}
COMMAND_PERIOD_S = 0.02
POSE_WAIT_S = 10.0
POSE_TIMEOUT_S = 0.3  # 15 EKF periods; about 9 deg or 30 mm driven blind
SETTLE_S = 1.5  # after stopping, so the report includes the coast to a stop
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"


class PoseTracker:
    """The latest pose, and the unwrapped turn since the first."""

    def __init__(self):
        self.start = None
        self.last = None
        self.turn_rad = 0.0

    def feed(self, x, y, yaw):
        if self.start is None:
            self.start = self.last = (x, y, yaw)
            return
        self.turn_rad += math.remainder(yaw - self.last[2], 2 * math.pi)
        self.last = (x, y, yaw)

    def distance_from(self, pose):
        """Straight-line distance, as Nav2's DriveOnHeading measures progress:
        the estimate's small sideways wobble at a stop doesn't add up."""
        return math.hypot(self.last[0] - pose[0], self.last[1] - pose[1])

    def offset_from_start(self):
        """(forward, left) in metres, along the start heading."""
        start_x, start_y, start_yaw = self.start
        x, y, _ = self.last
        dx, dy = x - start_x, y - start_y
        return (dx * math.cos(start_yaw) + dy * math.sin(start_yaw),
                -dx * math.sin(start_yaw) + dy * math.cos(start_yaw))


class NewPoses:
    """Feeds a tracker only poses with a new stamp, and knows when the last
    one arrived (monotonic seconds)."""

    def __init__(self, tracker):
        self.tracker = tracker
        self.stamp = None
        self.arrived_s = -math.inf

    def feed(self, stamp, x, y, yaw, now_s):
        if stamp == self.stamp:
            return
        self.stamp = stamp
        self.arrived_s = now_s
        self.tracker.feed(x, y, yaw)

    def is_stale(self, now_s):
        return now_s - self.arrived_s > POSE_TIMEOUT_S


def yaw_from_quaternion(z, w):
    """Yaw of a rotation about z only, as the robot's poses are."""
    return 2 * math.atan2(z, w)


def ramped_speed(current, remaining, cruise, accel, dt):
    """The next command: no faster than accel allows, slow enough to stop in
    what remains, and at least MIN_SPEED_FRACTION of cruise until the end."""
    stoppable = math.sqrt(2.0 * accel * max(remaining, 0.0))
    target = max(min(cruise, stoppable), MIN_SPEED_FRACTION * cruise)
    return min(target, current + accel * dt)


def moves_for(kind, amount):
    """The (kind, amount) moves to drive: one, or eight for a square."""
    if kind != "square":
        return [(kind, amount)]
    side = ("straight", abs(amount))
    corner = ("spin", math.copysign(90.0, amount))
    return [side, corner] * 4


def print_report(name, tracker):
    forward_m, left_m = tracker.offset_from_start()
    print(f"{name} after stopping:")
    print(f"  ended {forward_m:+.3f} m forward and {left_m:+.3f} m left of the "
          "start")
    print(f"  turned {math.degrees(tracker.turn_rad):+.1f} deg "
          "(positive is left)")


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("straight", "spin", "square"):
        sys.exit(__doc__)
    moves = moves_for(sys.argv[1], float(sys.argv[2]))

    # Imported here so the tests run without ROS sourced.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    # Ctrl+C, a closed terminal and kill all reach the stop in finally below.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for stop_signal in (signal.SIGHUP, signal.SIGTERM):
        signal.signal(stop_signal, signal.default_int_handler)
    node = rclpy.create_node("odom_drive")
    publisher = node.create_publisher(Twist, "cmd_vel", 10)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841, kept alive
    ekf_tracker = PoseTracker()  # what it steers by
    ekf_poses = NewPoses(ekf_tracker)
    wheel_tracker = PoseTracker()  # /odom alone, to check wheel calibration
    node.create_subscription(
        Odometry, "odom",
        lambda msg: wheel_tracker.feed(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w)),
        10)

    def read_pose():
        try:
            transform = tf_buffer.lookup_transform(ODOM_FRAME, BASE_FRAME,
                                                   Time())
        except TransformException:
            return
        stamp = transform.header.stamp
        position = transform.transform.translation
        rotation = transform.transform.rotation
        ekf_poses.feed((stamp.sec, stamp.nanosec), position.x, position.y,
                       yaw_from_quaternion(rotation.z, rotation.w),
                       time.monotonic())

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=max(end - time.monotonic(), 0.0))
            read_pose()

    def stop():
        """Holds zero long enough to include the coast to a stop."""
        for _ in range(int(SETTLE_S / COMMAND_PERIOD_S)):
            publisher.publish(Twist())
            spin_for(COMMAND_PERIOD_S)

    # Each turn ends at the planned total, so overshoots don't add up.
    planned_turn_rad = 0.0

    def drive_move(kind, amount):
        """Drives one move by the pose; False if it stopped early. Each move
        aims short by how far the robot carries on after it."""
        nonlocal planned_turn_rad
        is_straight = kind == "straight"
        cruise, accel, run_on = MOVE_LIMITS[kind]
        if is_straight:
            target = abs(amount)
            start = ekf_tracker.last

            def remaining():
                return target - run_on - ekf_tracker.distance_from(start)
        else:
            target = math.radians(abs(amount))
            planned_turn_rad += math.radians(amount)
            direction = math.copysign(1.0, amount)

            def remaining():
                return ((planned_turn_rad - ekf_tracker.turn_rad) * direction
                        - run_on)

        deadline = time.monotonic() + 2 * target / cruise + 5.0
        speed = 0.0
        while remaining() > 0:
            if ekf_poses.is_stale(time.monotonic()):
                print(f"Stopped early: no new pose for {POSE_TIMEOUT_S} s. Is "
                      "robot.launch.py running? Over WiFi, run this on the "
                      "Pi.")
                return False
            if time.monotonic() > deadline:
                print("Stopped early: the move took too long. Is a Pico stop "
                      "active? Check /pico/stop_reason.")
                return False
            speed = ramped_speed(speed, remaining(), cruise, accel,
                                 COMMAND_PERIOD_S)
            drive = Twist()
            if is_straight:
                drive.linear.x = math.copysign(speed, amount)
            else:
                drive.angular.z = math.copysign(speed, amount)
            publisher.publish(drive)
            spin_for(COMMAND_PERIOD_S)
        return True

    def wait_for_poses():
        give_up_at = time.monotonic() + POSE_WAIT_S
        while ((ekf_tracker.start is None or wheel_tracker.start is None)
               and time.monotonic() < give_up_at):
            spin_for(0.1)
        if ekf_tracker.start is None or wheel_tracker.start is None:
            sys.exit(f"No /odom or {ODOM_FRAME} -> {BASE_FRAME} transform: is "
                     "robot_description's robot.launch.py running?")

    wait_for_poses()
    try:
        for i, (kind, amount) in enumerate(moves):
            if i > 0:
                stop()  # a full stop before each next move
            if not drive_move(kind, amount):
                break
    except KeyboardInterrupt:
        print("Interrupted: stopping.")
    finally:
        stop()

    print_report(f"The pose ({ODOM_FRAME} -> {BASE_FRAME})", ekf_tracker)
    print_report("/odom (the wheels alone)", wheel_tracker)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
