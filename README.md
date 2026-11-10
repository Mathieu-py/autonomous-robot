# Autonomous Mobile Robot

A small differential-drive robot, with its software written layer by layer: motor firmware, wheel odometry and an EKF, LiDAR mapping, path planning, obstacle avoidance and frontier exploration. The goal is one unedited take where the robot maps a room it has never seen, then drives to any point clicked on the map.

It runs ROS 2 Jazzy on a Raspberry Pi 5, with a Raspberry Pi Pico 2 W handling motor control and an RPLIDAR C1 for mapping.

## Status

As of 9 November 2026:

- **Parts:** ordered and received.
- **Simulation:** ROS 2 Jazzy, Gazebo and Nav2 run under WSL2. The Nav2 TurtleBot demo reaches its goals at about real time.
- **Hardware:** every part passed its [bench tests](docs/bench-tests.md). The motor firmware runs PID speed control with bumper, low-battery, stall and watchdog stops, and drives from ROS 2 through keyboard teleop. Wiring, the chassis and moving ROS to the Pi are next.
- **Milestones:** [milestone 1](docs/milestone-1.md) (drive it) is finished on the bench, with the wheels lifted; driving on the floor waits for the chassis. Each milestone ends with a measurement written up in this repo.

## Documents

- [build-plan.md](docs/build-plan.md): the final demo, milestones, schedule, computing setup, safety and risks
- [parts-list.md](docs/parts-list.md): exact parts and prices, compatibility checks and wiring
- [bench-tests.md](docs/bench-tests.md): step-by-step checks for each part, with results
- [milestone-1.md](docs/milestone-1.md): milestone 1 write-up: speed control, safety stops and teleop, with measurements
- [results/](results/README.md): test data, logs and charts, such as PID step responses
- [tools/](tools/): helper scripts, such as `plot_pid.py` for charting a PID log
