# Autonomous Mobile Robot

A small differential-drive robot, with its software written layer by layer: motor firmware, wheel odometry and an EKF, LiDAR mapping, path planning, obstacle avoidance and frontier exploration. The goal is one unedited take where the robot maps a room it has never seen, then drives to any point clicked on the map.

It runs ROS 2 Jazzy on a Raspberry Pi 5, with a Raspberry Pi Pico 2 W handling motor control and an RPLIDAR C1 for mapping.

## Status

As of 6 November 2026:

- **Parts:** ordered and received.
- **Simulation:** ROS 2 Jazzy, Gazebo and Nav2 run under WSL2. The Nav2 TurtleBot demo reaches its goals at about real time.
- **Hardware:** the parts passed their [bench tests](bench-tests.md), apart from the safety checks that need firmware. Wiring, the chassis and the motor firmware are next.
- **Milestones:** none finished yet. Each one ends with a measurement written up in this repo.

## Documents

- [build-plan.md](build-plan.md): the final demo, milestones, schedule, computing setup, safety and risks
- [parts-list.md](parts-list.md): exact parts and prices, compatibility checks and wiring
- [bench-tests.md](bench-tests.md): step-by-step checks for each part, with results
