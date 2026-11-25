"""Drive a set distance or angle by /odom, stop, and print where /odom says the
robot ended, to compare with a tape measure.

Usage (ROS 2 sourced; the bridge and wheel_odometry running):
  python3 tools/odom_drive.py straight <metres>   # 0.1 m/s; negative reverses
  python3 tools/odom_drive.py spin <degrees>      # on the spot at 0.5 rad/s;
                                                  # negative spins right
"""
import math
import sys
import time

LINEAR_MPS = 0.1
ANGULAR_RADPS = 0.5
COMMAND_PERIOD_S = 0.05
ODOM_WAIT_S = 10.0
SETTLE_S = 1.5  # after stopping, so the report includes the coast to a stop


class OdomTracker:
    """Distance and unwrapped turn since the first pose it was fed."""

    def __init__(self):
        self.start = None
        self.last = None
        self.distance_m = 0.0
        self.turn_rad = 0.0

    def feed(self, x, y, yaw):
        if self.start is None:
            self.start = self.last = (x, y, yaw)
            return
        last_x, last_y, last_yaw = self.last
        self.distance_m += math.hypot(x - last_x, y - last_y)
        self.turn_rad += math.remainder(yaw - last_yaw, 2 * math.pi)
        self.last = (x, y, yaw)

    def offset_from_start(self):
        """(forward, left) in metres, along the start heading."""
        start_x, start_y, start_yaw = self.start
        x, y, _ = self.last
        dx, dy = x - start_x, y - start_y
        return (dx * math.cos(start_yaw) + dy * math.sin(start_yaw),
                -dx * math.sin(start_yaw) + dy * math.cos(start_yaw))


def yaw_from_quaternion(z, w):
    """Yaw of a rotation about z only, as wheel_odometry publishes."""
    return 2 * math.atan2(z, w)


def main():
    if len(sys.argv) != 3 or sys.argv[1] not in ("straight", "spin"):
        sys.exit(__doc__)
    is_straight = sys.argv[1] == "straight"
    amount = float(sys.argv[2])
    target = abs(amount) if is_straight else math.radians(abs(amount))
    speed = LINEAR_MPS if is_straight else ANGULAR_RADPS

    # Imported here so the tests run without ROS sourced.
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.signals import SignalHandlerOptions

    # Ctrl+C stays a KeyboardInterrupt, so the stop below still gets sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("odom_drive")
    publisher = node.create_publisher(Twist, "cmd_vel", 10)
    tracker = OdomTracker()
    node.create_subscription(
        Odometry, "odom",
        lambda msg: tracker.feed(
            msg.pose.pose.position.x, msg.pose.pose.position.y,
            yaw_from_quaternion(msg.pose.pose.orientation.z,
                                msg.pose.pose.orientation.w)),
        10)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=end - time.monotonic())

    def progress():
        return tracker.distance_m if is_straight else abs(tracker.turn_rad)

    give_up_at = time.monotonic() + ODOM_WAIT_S
    while tracker.start is None and time.monotonic() < give_up_at:
        rclpy.spin_once(node, timeout_sec=0.1)
    if tracker.start is None:
        sys.exit("No /odom: is robot_description's robot.launch.py running?")

    drive = Twist()
    if is_straight:
        drive.linear.x = math.copysign(LINEAR_MPS, amount)
    else:
        drive.angular.z = math.copysign(ANGULAR_RADPS, amount)
    deadline = time.monotonic() + 2 * target / speed + 5.0
    try:
        while progress() < target:
            if time.monotonic() > deadline:
                print("Stopped early: no progress. Is a Pico stop active? "
                      "Check the bridge's journal.")
                break
            publisher.publish(drive)
            spin_for(COMMAND_PERIOD_S)
    except KeyboardInterrupt:
        print("Stopped by Ctrl+C.")
    finally:
        for _ in range(int(SETTLE_S / COMMAND_PERIOD_S)):
            publisher.publish(Twist())
            spin_for(COMMAND_PERIOD_S)

    forward_m, left_m = tracker.offset_from_start()
    print("/odom after stopping:")
    print(f"  travelled {tracker.distance_m:.3f} m; ended {forward_m:+.3f} m "
          f"forward and {left_m:+.3f} m left of the start")
    print(f"  turned {math.degrees(tracker.turn_rad):+.1f} deg "
          "(positive is left)")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
