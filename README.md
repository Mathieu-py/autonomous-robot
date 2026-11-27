# Autonomous Mobile Robot

A small differential-drive robot, with its software written layer by layer: motor firmware, wheel odometry and an EKF, LiDAR mapping, path planning, obstacle avoidance and frontier exploration. The goal is one unedited take where the robot maps a room it has never seen, then drives to any point clicked on the map.

It runs ROS 2 Jazzy on a Raspberry Pi 5, with a Raspberry Pi Pico 2 W handling motor control and an RPLIDAR C1 for mapping.

## Status

As of 27 November 2026:

- **Parts:** ordered and received.
- **Simulation:** ROS 2 Jazzy, Gazebo and Nav2 run under WSL2. The Nav2 TurtleBot demo reaches its goals at about real time.
- **Hardware:** every part passed its [bench tests](docs/bench-tests.md). The robot is built and drives on the floor on its own battery: PID speed control with bumper, low-battery, stall and watchdog stops on the Pico, and ROS 2 on the Pi, which powers itself off when the battery runs low.
- **Milestones:** [milestone 1](docs/milestone-1.md) (drive it), [milestone 2](docs/milestone-2.md) (know where it is) and [milestone 3](docs/milestone-3.md) (see) are finished: after a 1 m square, the robot's own EKF knows where it ended to within 0.2° and 8 mm, and its first map of a room, from the LiDAR and SLAM Toolbox, matches a tape measure to within 4 cm. Each milestone ends with a measurement written up in this repo.

## Documents

- [build-plan.md](docs/build-plan.md): the final demo, milestones, schedule, computing setup, safety and risks
- [parts-list.md](docs/parts-list.md): exact parts and prices, compatibility checks and wiring
- [bench-tests.md](docs/bench-tests.md): step-by-step checks for each part, with results
- [pi-setup.md](docs/pi-setup.md): ROS 2 on the robot's Pi, the Pico's USB link, and networking with the laptop
- [milestone-1.md](docs/milestone-1.md): milestone 1 write-up: speed control, safety stops and teleop, with measurements
- [milestone-2.md](docs/milestone-2.md): milestone 2 write-up: wheel odometry, our EKF against robot_localization, and the square tests
- [milestone-3.md](docs/milestone-3.md): milestone 3 write-up: the LiDAR and its mount, the laser's frame, and the first map checked with a tape measure
- [results/](results/README.md): test data, logs and charts, such as PID step responses
- [tools/](tools/): helper scripts, such as `plot_pid.py` for charting a PID log
