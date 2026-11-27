# Milestone 2: Know where it is

Finished on 26 November 2026, on the floor and on the battery. The robot tracks where it is from its wheels and gyro with its own EKF. After a 1 m square, our EKF's estimate is within 0.2–0.4° and 8 mm of where the robot really stopped, against robot_localization's 0.8–1.0° and 16–25 mm (runs 04 and 05).

**Done when** ([build plan](build-plan.md)): drift measured after driving a square, and your EKF compared with robot_localization.

| Check | Measured (six 1 m squares, three each way) | Data |
| --- | --- | --- |
| Drift after a square, our EKF | Counted 0.17° too much turn per square (spread 0.04°); ended 7.9 mm from where the robot really stopped (worst 14 mm) | [Square run 04](../results/README.md#square-tests-squares) |
| The same, robot_localization | 1.04° too little (spread 1.36°); 24.9 mm (worst 44 mm) | Square run 04 |
| The same, the wheels alone (`/odom`) | 1.52° too little (spread 1.27°); 28.2 mm (worst 51 mm) | Square run 04 |
| Two turns on the spot | Our EKF 0.02% too much; robot_localization 0.42% and `/odom` 0.74% too little | Square run 04 |
| Driving by its own estimate | Steered by our EKF, the robot ended each square 1.5 mm and 0.08° from its start, on average | Square run 04 |

Four square sessions got there, one change at a time ([square tests](../results/README.md#square-tests-squares)):

| Run | What changed | Our EKF, per square | robot_localization |
| --- | --- | --- | --- |
| 01 | First version of our EKF | 3.11° too little, 29.7 mm | 2.16° too little, 24.8 mm |
| 02 | Gyro offset learned only while not turning | 1.28° too little, 12.5 mm | 2.05° too little, 27.3 mm |
| 03 | Steering by our EKF, on the Pi | 0.75° too much, 21.2 mm | 0.92° too little, 33.2 mm |
| 04 | Gyro filter 10 Hz instead of 44 Hz | 0.17° too much, 7.9 mm | 1.04° too little, 24.9 mm |

**Rechecked the next day** ([run 05](../results/README.md#square-tests-squares), 27 November): two turns on the spot each way read as on 26 November, our EKF 0.027% too much against 0.02%, so the 10 Hz filter fixed the gyro's error changing between days. One square each way: our EKF 0.44° too much turn per square, and 8.0 mm from where the robot really stopped.

## What was built

**Wheel odometry** ([ros/wheel_odometry/](../ros/wheel_odometry/)): turns `/joint_states` into `/odom`. The wheel size and spacing come from driving on the floor, not the calipers: the wheels roll as if 67.0 mm across and turn as if 170.4 mm apart ([odometry calibration](../results/README.md#odometry-calibration-odometry)). Its covariances are the measured noise ([odometry and IMU noise](../results/README.md#odometry-and-imu-noise)).

**robot_localization, as the yardstick** ([config/robot_localization.yaml](../ros/robot_description/config/robot_localization.yaml)): fuses `/odom`'s speed and turn rate with the gyro's turn rate into `/odometry/filtered`.

**Our EKF** ([ros/robot_ekf/](../ros/robot_ekf/)): a pure C++ class with no ROS ([ekf.hpp](../ros/robot_ekf/include/robot_ekf/ekf.hpp)), and a node around it.

- It estimates position, heading, speed, turn rate and the gyro's zero offset, which can reach a few degrees a minute and drifts.
- It learns the offset only while the robot isn't turning. At each start and stop of a turn, the body turns further than the wheels count, and an EKF free to explain that as offset follows the wheels instead (run 01).
- It ignores the wheels' turn rate while they slip, when they disagree with the gyro for 8 readings in a row. Starting and stopping disagree for up to 5.
- The node sorts the two sensors' readings by time (about 40 ms behind), restarts if time jumps back, and publishes `/ekf/odom`, `/ekf/gyro_bias` and the `odom → base_link` transform, the pose Nav2 will steer by. Only one node may publish that transform ([REP 105](https://www.ros.org/reps/rep-0105.html)), so `wheel_odometry` doesn't publish it and `robot_localization.yaml` turns robot_localization's off.
- 19 unit tests: the EKF on straight runs, turns, slips, offsets and time gaps, and the reading sorter.

**A test driver that drives like Nav2** ([tools/odom_drive.py](../tools/odom_drive.py)): drives a set distance, angle or square by the `odom → base_link` transform, as Nav2's Spin and DriveOnHeading behaviours do. It ramps its speed with Nav2's velocity smoother defaults (2.5 m/s², 3.2 rad/s²) and slows so it could stop in what remains. Each move stops short by how far the robot carries on, 14 mm and 3.0°, as ROS By Example's `angular_tolerance` does. It runs on the Pi.

**A gyro filter that doesn't alias** ([firmware/src/devices/imu.cpp](../firmware/src/devices/imu.cpp)): the MPU-6050's filter is now 10 Hz instead of 44 Hz, below half the 50 Hz rate the firmware reads it at.

**Measuring tools:** `tools/odom_imu_noise.py` for the sensors' noise, and `tools/end_pose_errors.py` to compare each estimate with the floor after each square.

## What went wrong and what it taught

- **A lost recording.** After a battery swap, the SSH session for recording had dropped to a laptop shell, so the recording ran on the laptop. Copying the Pi's folder of the same name to the laptop then overwrote it. Check the prompt says `robot-pi` before recording, and copy recordings to a new name.
- **The first EKF followed the wheels.** With the gyro's offset in its state, it put the gyro's extra turn at each start and stop down to offset, and missed as much turn as the wheels did (3.1° per square). Learning the offset only while not turning more than halved the error (run 02).
- **The body turns further than the wheels count.** At each start and stop of a turn the body moves a little the encoders don't see, probably the gearbox's backlash and the tyres' give, since the encoders are on the motor side. Two full turns on the spot have only one start and one stop, so the wheel calibration barely sees it; only the gyro does.
- **The robot carries on after each stop.** With the firmware's own ramp alone, a turn stopped at about 12 rad/s², a jolt. Ramping like Nav2's velocity smoother more than halved how far each turn carried on after its last command. Stopping short by the measured carry-on, as ROS By Example does, landed the moves on target.
- **Steering over WiFi.** Driven from the laptop, the pose arrives late, so the robot stops late and inconsistently: one turn stopped 3.2° past its target. Steering on the Pi, as Nav2 does, made the stops consistent; the laptop only sends things that can arrive late, like goals and teleop keys.
- **Two errors that cancelled.** In run 03 the robot came back within 0.12° of its start heading, but only because our EKF counted 0.75° too much turn per square and each turn carried on about as far. Fixing one would have exposed the other, so each was measured and fixed on its own.
- **A gyro that aliased.** The gyro's error changed from session to session: about 0.4% low on 25 November's spins, about right in run 02, 0.4% high in run 03. The firmware reads it at 50 Hz, but its filter passed up to 44 Hz, so motor vibration between 25 and 44 Hz folded into false slow readings. A 10 Hz filter cut the variance of its noise while driving to a third, and our EKF's turn error from 0.75° to 0.17° per square, at the cost of 9 ms of delay.
- **Recordings on two clocks.** Recorded on the laptop, `/cmd_vel` carries the laptop's time, 1.5 s ahead of the Pi's. `end_pose_errors.py` times each command by the `/odom` recorded just before it instead.

## Tried and not kept

Each was tested by replaying a recording through the `Ekf` class with one thing changed, and compared with the floor. The replays start the EKF afresh, so they differ slightly from the live results.

| Change | Our EKF, per square | Why not |
| --- | --- | --- |
| None (as built), run 04 | 0.17° too much, 8.3 mm | |
| Learn the gyro's offset only while still, run 03 | Spread 1.5–2° instead of 0.09° | The pauses between moves are too short to follow an offset that drifts within a square |
| Rescale the gyro to run 03's floor results | 0.53° too little, 16 mm | The gyro's scale wasn't the same in run 02; aliasing was the cause |
| Tell it the gyro's lower noise with the 10 Hz filter, run 04 | 0.48° too much, 11 mm | The noise setting also covers the gyro's errors that aren't noise, such as its drifting offset |
| Correct the wheel spacing for `/odom`'s 0.74% on spins, run 04 | 0.36–0.50° too much, 10–11 mm | The gyro reads slightly high and the wheels low, and the EKF's blend lands closest to the truth |
| Steer by the gyro alone, run 04 | 2.0° too much, 41 mm | The wheels are what let the EKF follow the gyro's offset |

## Not done yet

- **`/odom` misses 0.65–0.74% of two turns on the spot** (runs 04 and 05), against 0.2% after the floor calibration on 25 November. It doesn't affect our EKF.
- **Our EKF's end still lands a few millimetres outside the square,** after every square, both ways; 20 mm in run 03, under 10 mm in runs 04 and 05.
- **Turns that carried on further.** In run 05's right square every turn ran 0.66–1.05° past its target, against about 0.25° in run 04, and our EKF counted 0.55° too much turn in that square. One square each way can't tell a trend from one bad square; the next square test will show.
- **Hard floor only.** Carpet, and the extra weight of the LiDAR, haven't been tested; the square test is quick to repeat for both.
