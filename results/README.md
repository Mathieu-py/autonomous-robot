# Results

Test data, logs and charts from the robot. Bench-test procedures and pass/fail results are in [docs/bench-tests.md](../docs/bench-tests.md); this folder holds the raw data behind firmware tests.

## Naming

`YYYY-MM-DD-runNN-<load>-<speed>-<what changed>`, with the CSV and its chart sharing a name. Run numbers count up within each folder and are zero-padded so they sort in order. `<load>` is `no-load` (wheels lifted) or `fingers-on` (light finger pressure on both wheels); `<speed>` is the test target in revolutions per second; `<what changed>` names the change since the previous run.

CSVs are kept exactly as the firmware logged them, header included.

## PID speed loop ([pid/](pid/))

Logged at 50 Hz in the firmware's speed test mode (`kMode = Mode::kSpeedTest`, [firmware/src/bench/motor_tests.cpp](../firmware/src/bench/motor_tests.cpp)). Columns: `t_ms` (since the test started), `target` (rev/s, see below), then each wheel's measured speed (rev/s, the average over the last 20 ms) and motor power (PWM fraction, -0.95 to +0.95). Plot with:

```bash
python3 tools/plot_pid.py <input.csv> <output.png> "<title>"
```

Every run: 12 V bench supply with a 1.5 A limit, and the test sequence target forward for 3 s, stop for 1 s, minus target for 3 s.

**What `target` means changed between runs:**

- Runs 01–03: the commanded target, which jumps in a single step.
- Runs 04–06: the ramped setpoint for the end of the coming 20 ms. Each measured speed covers the previous 20 ms, so the two are about 30 ms apart, and during ramps the charts make the wheels look about 0.15 rev/s behind when they were really on the ramp or slightly ahead of it.
- Runs 07–08: the ramped setpoint at the middle of the 20 ms the speed was measured over, so the two line up in time.

From run 09 on, the firmware names this column `setpoint` instead of `target`; it means the same as in runs 07–08.

| Run | Controller | Load, target | Result |
| --- | --- | --- | --- |
| [01](pid/2026-11-09-run01-no-load-0.5rps-first-gains.png) ([CSV](pid/2026-11-09-run01-no-load-0.5rps-first-gains.csv)) | Feedforward 0.35, kP 0.3, kI 1.5 | No load, 0.5 rev/s | Reached 0.5 rev/s in about 70 ms and settled in about 120 ms; one sample overshot to 0.566. Both wheels averaged 0.50. |
| [02](pid/2026-11-09-run02-fingers-on-0.5rps-first-gains.png) ([CSV](pid/2026-11-09-run02-fingers-on-0.5rps-first-gains.csv)) | As run 01 | Fingers on, 0.5 rev/s | Both wheels still averaged 0.50. The second wheel needed about 0.22 power, against about 0.19 unloaded, and sat at 0.47–0.48 rev/s for about 0.5 s before the integral caught up: kI on the slow side. |
| [03](pid/2026-11-09-run03-no-load-1.0rps-higher-ki.png) ([CSV](pid/2026-11-09-run03-no-load-1.0rps-higher-ki.csv)) | kI raised to 3.0 | No load, 1.0 rev/s | Steady speed exact (1.001 rev/s at 0.350–0.356 power), but it overshot to 1.19 (19%) and took about 600 ms to settle: the integral builds up during the sudden step and then unwinds slowly. |
| [04](pid/2026-11-09-run04-no-load-1.0rps-ramp.png) ([CSV](pid/2026-11-09-run04-no-load-1.0rps-ramp.csv)) | Plus a 5 rev/s² ramp on the target | No load, 1.0 rev/s | Overshoot down from 19% to 11% (peak 1.11 in both directions); about 500 ms to settle. The chart's apparent lag behind the ramp is the logging offset described above. |
| [05](pid/2026-11-09-run05-no-load-1.0rps-accel-feedforward.png) ([CSV](pid/2026-11-09-run05-no-load-1.0rps-accel-feedforward.csv)) | Plus acceleration feedforward, kA 0.025 (hand estimate) | No load, 1.0 rev/s | Settled in about 280 ms instead of 500, but the peak stayed at 1.11. At the time this was put down to kA; run 06 showed the real cause was timing. |
| [06](pid/2026-11-09-run06-no-load-1.0rps-measured-gains.png) ([CSV](pid/2026-11-09-run06-no-load-1.0rps-measured-gains.csv)) | Measured per motor (see motor identification): kS 0.01, kV 0.35, kA 0.017, kP 0.28, kI 5.5 | No load, 1.0 rev/s | Same 1.11–1.13 peak, so kA was not the cause. The real cause was timing: the controller compared each speed reading, an average over the last 20 ms, with the setpoint for the end of the next 20 ms, so during ramps it pushed the wheels about 0.15 rev/s ahead. A simulation using the measured motor model reproduced this run (peak 1.119, settled about 260 ms). |
| [07](pid/2026-11-09-run07-no-load-1.0rps-aligned-timing.png) ([CSV](pid/2026-11-09-run07-no-load-1.0rps-aligned-timing.csv)) | As run 06, with the timing aligned | No load, 1.0 rev/s | Peak 1.011 in both directions, one encoder step (0.02 rev/s) above the target, so no real overshoot. The wheels follow the ramp within 0.035 rev/s and are settled when it ends, about 230 ms after the command. Steady speed 1.000 on both wheels, matching the simulation. |
| [08](pid/2026-11-09-run08-fingers-on-1.0rps-aligned-timing.png) ([CSV](pid/2026-11-09-run08-fingers-on-1.0rps-aligned-timing.csv)) | As run 07 | Fingers on, 1.0 rev/s | Held both wheels at 1.000 rev/s, the second taking about 0.38 power against 0.35 unloaded. Worst sample 3% slow and none beyond 4%, against run 02's worst of 7% with 3 samples beyond 4% (as a fraction of each run's target; finger pressure can't be matched exactly between runs). No visible dips. |
| [09](pid/2026-11-09-run09-no-load-1.0rps-refactor-check.png) ([CSV](pid/2026-11-09-run09-no-load-1.0rps-refactor-check.csv)) | As run 07, after splitting the firmware into modules | No load, 1.0 rev/s | Same as run 07: peak 1.011 in both directions, steady 1.000 on both wheels at 0.349 power (0.347 in run 07), worst ramp tracking error 0.033 (0.035). The refactor didn't change the behaviour. |
| [10](pid/2026-11-09-run10-no-load-1.0rps-naming-and-format-check.png) ([CSV](pid/2026-11-09-run10-no-load-1.0rps-naming-and-format-check.csv)) | As run 09, after the naming pass and clang-format | No load, 1.0 rev/s | Same as runs 07 and 09: peak 1.011, steady 1.000 at 0.349 power, worst ramp tracking error 0.023. |

Runs 01 and 03 were copied from the serial output in the terminal; the rest are from PuTTY logs.

## Motor identification ([sysid/](sysid/))

Open-loop test from the firmware's identification mode (`kMode = Mode::kSysIdTest`): slow power ramps to ±0.6 over 10 s, then steps to ±0.5 power, logged at 50 Hz with columns `t_ms`, `phase`, `power`, `first_speed`, `second_speed`. Fit with:

```bash
python3 tools/fit_motor.py <log.csv> <chart.png>
```

The script also saves its printed numbers next to the chart, as a `.txt` with the same name.

The model is `power = kS * sign(v) + kV * v + kA * a`, as in WPILib SysId. kS and kV come from the slow ramps, and kA from the step response's time constant (kA = kV * tau). PI gains use IMC (lambda) tuning: kP = kA / (lambda + delay), kI = kV / (lambda + delay).

| Run | Motor | kS (power) | kV (power per rev/s) | kA (power per rev/s²) | Time constant | Dead time | Gains used (lambda = 2 x delay) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| [01](sysid/2026-11-09-run01-no-load-both-motors.png) ([CSV](sysid/2026-11-09-run01-no-load-both-motors.csv), [numbers](sysid/2026-11-09-run01-no-load-both-motors.txt)) | First | 0.012 | 0.345 | 0.0174 | 50 ms | 11 ms | kP 0.277, kI 5.48 |
| | Second | 0.010 | 0.349 | 0.0171 | 49 ms | 11 ms | kP 0.272, kI 5.54 |

Wheels lifted, 12 V bench supply with a 1.5 A limit. The ramp fits have an RMS error of about 0.005 power. At 0.5 power the first wheel ran 2.9% faster forward than backward (1.454 against 1.413 rev/s); the second only 0.4% (1.432 against 1.425). The fit averages each wheel's two directions, and the integral term absorbs the rest. The 11 ms dead time is mostly the speed reading averaging over each 20 ms window. The hand estimate of kA used in run 05, 0.025, was about 45% too high.

## Safety checks ([safety/](safety/))

Logged at 50 Hz in the firmware's safety test mode (`kMode = Mode::kSafetyTest`): both wheels run at 1.0 rev/s for 8 s, then stop for 2 s, repeating for 60 s, so each stop can clear before the next check. Columns as the PID logs, plus `battery_volts` (filtered) and `stop` (the active stop reason). Bench test 6 in [docs/bench-tests.md](../docs/bench-tests.md) describes the checks. Plot with `python3 tools/plot_safety.py <input.csv> <output.png> "<title>"`.

| Run | Checks done | Result |
| --- | --- | --- |
| [01](safety/2026-11-09-run01-no-load-1.0rps-bumpers-and-holds.png) ([CSV](safety/2026-11-09-run01-no-load-1.0rps-bumpers-and-holds.csv)) | Bumpers, several times; a wheel held by hand | Every bumper press stopped both motors at once and cleared at the next pause. Holding a wheel saturated its power at 0.95 and hit the bench supply's 1.5 A limit; the supply's voltage then sagged from 11.9 to about 10.7 V and the low-battery cutoff stopped the motors, about 1 s in. The wheel was still turning at about 0.45 rev/s, so the stall check (below 0.05 rev/s for 0.5 s) didn't trigger. Not yet tested: the low-battery cutoff from a deliberate voltage change, and the stall check. |
| [02](safety/2026-11-09-run02-no-load-1.0rps-battery-and-stall.png) ([CSV](safety/2026-11-09-run02-no-load-1.0rps-battery-and-stall.csv)) | Supply lowered to 10.3 V; wheels held by hand; an encoder's ground unplugged (not in this log) | Lowering the supply stopped both motors at 10.50 V (filtered) with the wheels at full speed, and it cleared once the supply was back above 11.0 V. Each firm hold stopped the wheel within about 0.3 s, through the low-battery cutoff again: the 1.5 A limit sags the supply faster than the 0.5 s stall check. Unplugging an encoder's ground mid-run, so its speed reads 0 at high power, triggered the stall check (seen in PuTTY; not in this log). |

Wheels lifted, 12 V bench supply with a 1.5 A limit, battery voltage read through the 33k/10k divider from the same supply.

## Teleop ([teleop/](teleop/))

Keyboard driving through `pico_bridge` (`ros/pico_bridge`), recorded with `ros2 bag record /cmd_vel /joint_states /battery /rosout` into `~/robot_ws/bags/` (outside the repo). Convert with `python3 tools/plot_teleop.py <bag dir> <output.csv> <output.png> "<title>" 0.068 0.20` (ROS 2 sourced); the drive geometry comes from `ros/robot_description/config/robot.yaml`, and the trailing `0.068 0.20` overrides it with the wheel diameter and separation these runs were driven with. Columns: `t_ms` (counted from the first wheel reading, 20 ms per Pico status), each wheel's commanded and measured speed (rev/s; commands rebuilt from `/cmd_vel` with the bridge's 1 s timeout), and `battery_volts`. Left wheel is the first motor, neither reversed. Every run: wheels lifted, 12 V bench supply with a 1.5 A limit, keyboard at 0.1 m/s and 0.5 rad/s.

**WSL's clock steps backwards.** Inside WSL the system clock runs fast and is pulled back to Windows time about every 30 s, by 0.6–2.4 s. Every ROS timestamp on the laptop jumps back with it, so a recording read in timestamp order interleaves the next second or two with the previous one: in the first charts this looked like the wheels flicking between two speeds. The data itself was always in order (a live subscriber saw no reordering, and a fake Pico on a pseudo-terminal reproduced it with no hardware). `plot_teleop.py` therefore reads recordings in file order and counts time from the Pico's 20 ms status rate. Disabling Ubuntu's own time sync (`systemd-timesyncd`) roughly halved the jumps but didn't remove them. The Pi runs native Ubuntu, so this only affects the laptop.

| Run | Driving | Result |
| --- | --- | --- |
| [01](teleop/2026-11-09-run01-no-load-keyboard.png) ([CSV](teleop/2026-11-09-run01-no-load-keyboard.csv)) | Forward, backward, turns on the spot and arcs | Both wheels followed every command. |
| [02](teleop/2026-11-09-run02-no-load-keyboard-and-bridge-stop.png) ([CSV](teleop/2026-11-09-run02-no-load-keyboard-and-bridge-stop.csv)) | As run 01, then the bridge stopped mid-drive | Both wheels followed the commands. Stopping the bridge (Ctrl+C) mid-drive stopped the wheels through the Pico's 250 ms command timeout, as seen on the bench; the recording ends with the bridge, so it isn't in the data. |
| [03](teleop/2026-11-09-run03-no-load-keyboard-and-bumper.png) ([CSV](teleop/2026-11-09-run03-no-load-keyboard-and-bumper.csv)) | Driving with two bumper presses | Each press stopped both wheels while the command continued; the bridge logged the stop and the clear, and driving resumed. |
| [04](teleop/2026-11-09-run04-no-load-keyboard-quick-switching.png) ([CSV](teleop/2026-11-09-run04-no-load-keyboard-quick-switching.csv)) | 60 s with quick key switching | Both wheels followed every change. |

## IMU ([imu/](imu/))

The MPU-6050 published by `pico_bridge` on `/imu/data_raw` at 50 Hz (±4 g, ±500 °/s, ~44 Hz filter; 10 Hz since 26 November, see [square tests](#square-tests-squares)), recorded on the Pi with `ros2 bag record /imu/data_raw /joint_states` into `~/bags/` (outside the repo). Convert with `python3 tools/imu_bag_to_csv.py <bag dir> <output.csv>` (ROS 2 sourced). Columns: `t_s` (from the first message), `ax ay az` (m/s²) and `gx gy gz` (rad/s), in the chip's axes (X robot right, Y forward, Z up), with the bridge's start-up gyro offset already removed. Every run: on the battery, motor switch off, on a table.

| Run | Test | Result |
| --- | --- | --- |
| [01](imu/2026-11-23-run01-still-172s.csv) | 172 s untouched | Gyro variance 3.7–4.5 × 10⁻⁷ (rad/s)², accel variance 1.9–5.3 × 10⁻⁴ (m/s²)²; the largest accel one is the bridge's covariance default (its gyro default is the driving value, under odometry noise below). Gravity reads 9.29 m/s², 5.6% low. The gyro Z offset left after the 1 s start-up measurement drifted between −0.02 and −0.06 °/s, which is −5.5° of heading over the run if uncorrected. Samples 20.0 ± 0.2 ms apart. |
| [02](imu/2026-11-23-run02-three-left-turns-by-hand.csv) | Three left turns by hand, from and back to a straight edge | 1082.7° integrated for 1080° (0.25%, within how well the turns could be lined up by hand), at up to 98 °/s. No yaw scale correction needed at this resolution. |

## Odometry calibration ([odometry/](odometry/))

`tools/odom_drive.py` drove a set distance or angle by `/odom`, stopped, and printed where `/odom` thought the robot ended; each run was then measured on the floor. (It now steers by our EKF's `odom → base_link` and prints `/odom` alongside.) Straight runs went 1 m along a strip of masking tape, measured at a flag on the deck's front centre (and one on its rear centre, from run 2). Spins were two full turns on the spot; the flag's straight-line distance d from its starting tick gives the extra angle, 2·asin(d / 2R) with R = 120 mm, the deck's radius. Every run: on the battery (11.35 V), on a hard floor, bumper arc not fitted yet. Each run and its reading: [2026-11-25-floor-calibration.csv](odometry/2026-11-25-floor-calibration.csv).

| Wheels in robot.yaml | Straight 1 m: measured ÷ `/odom` | Two turns: measured ÷ `/odom` |
| --- | --- | --- |
| Caliper: 68.0 mm diameter, 180 mm apart (runs 1–7) | 0.983–0.987 (3 runs) | 1.040–1.042 (4 runs, left and right alike) |
| Floor: 67.0 mm, 170.4 mm apart (runs 8–12) | 0.998 (2 runs) | 1.001–1.002 (3 runs) |

The floor values come from the averages of runs 1–7: the wheels roll as if they were 67.0 mm across, probably because the tyres flatten under the robot's weight, and the robot turns as if they were 170.4 mm apart, not the 180 mm between the tyre centres, probably because the tyres grip nearer their inner edges. The tape readings were to the nearest 1/8 in (3 mm, 0.3% of a metre), so the remaining 0.2% is at the limit of what these runs can show. The sideways drift on the straight runs went both ways (29 mm right to 3 mm left) and didn't turn the robot, so it came from lining up the start, not from unequal wheels.

## Odometry and IMU noise

Recorded on the Pi with `ros2 bag record -o ~/bags/odom_noise_2026-11-25 /odom /joint_states /imu/data_raw /cmd_vel /battery` (outside the repo): 30 s untouched, then `tools/odom_drive.py` driving 1 m forward and back twice, and two full turns each way twice, with the floor-calibrated wheels. One more drive ran with the motor switch off, so it didn't move and doesn't count. Measured with `python3 tools/odom_imu_noise.py <bag dir>` (ROS 2 sourced), which drops each run's first second and last half second and takes each reading's spread about that run's own average.

| Reading | Still | Straight at 0.1 m/s | Spinning at 0.5 rad/s |
| --- | --- | --- | --- |
| `/odom` speed | 0 | 1.5 × 10⁻³ m/s (variance 2.2 × 10⁻⁶) | 1.4 × 10⁻³ m/s |
| `/odom` turn rate | 0 | 1.8 × 10⁻² rad/s (variance 3.1 × 10⁻⁴) | 1.7 × 10⁻² rad/s |
| Gyro z | 5.6 × 10⁻⁴ rad/s (variance 3.2 × 10⁻⁷) | 2.0 × 10⁻² rad/s (variance 4.1 × 10⁻⁴) | 1.8 × 10⁻² rad/s |

- **The odometry's spread is the encoders' rounding.** One count in a 20 ms reading is 4.3 mm/s of wheel speed, and the two wheels' counts differ by one now and then, which is 0.025 rad/s of turn rate. It isn't real wobble: the odometry's and the gyro's turn rates differ by 2.8 × 10⁻² rad/s, the spread of two independent readings.
- **The gyro is about 35 times noisier while driving than still,** from the motors' vibration.
- **The covariance defaults are the straight-run values:** `wheel_odometry`'s `linear_velocity_variance` 2.2 × 10⁻⁶ and `angular_velocity_variance` 3.1 × 10⁻⁴, and `pico_bridge`'s `gyro_variance` 4.1 × 10⁻⁴. The still value it replaced, 4.5 × 10⁻⁷, made the gyro look 1,000 times better than it is while driving.
- **The gyro reads about 0.4% low.** It read 0.35–0.41% less than `/odom` on all four spins (720.4° against 722.9°, for example), and `/odom` is within about 0.1% of the floor measurements. The hand turns in IMU run 02 read 0.25% high. Both are well within the chip's ±3% sensitivity tolerance, so the gyro isn't rescaled. This was with the 44 Hz filter, which let motor vibration alias; with the 10 Hz filter the gyro agrees with the floor (square tests, run 04).

## Square tests ([squares/](squares/))

The drift test for milestone 2, after UMBmark. `python3 tools/odom_drive.py square 1.0` drives four 1 m sides at 0.1 m/s, each followed by a 90° turn on the spot at 0.5 rad/s, to the left (`square -1.0` turns right), stopping fully between moves. The deck carries a flag on its front edge and one on its rear edge, 240 mm apart with the turning centre midway, each set on a tape mark at the start. Afterwards each flag's distance from its own mark is measured in mm, along (+ ahead) and across (+ right). The robot's turn past whole turns is atan2(rear across − front across, 240 + front along − rear along), and where it ended is the flags' midpoint.

Each session was recorded with `ros2 bag record /odom /odometry/filtered /ekf/odom /imu/data_raw /cmd_vel /ekf/gyro_bias /battery /joint_states` into `~/bags/` (outside the repo) and compared with `python3 tools/end_pose_errors.py <bag dir> results/squares/<floor csv>` (ROS 2 sourced). Each floor CSV lists the measured drives by their order in the recording, counting from 0, with the four flag readings. The tool prints each estimate's turn and end error for each drive, and how far each turn and side ran past its target by our EKF. Files are named by date, run number and what changed. Every session: three squares each way (run 05: one), on the battery, on a hard floor.

| Run | What changed | Robot ended, from its start | Our EKF | robot_localization | `/odom` |
| --- | --- | --- | --- | --- | --- |
| [01](squares/2026-11-25-run01-first-ekf.csv) | First version of our EKF; `odom_drive.py` steers by `/odom`, from the laptop | 72 mm, 5.7° | −3.11° (spread 1.22), 29.7 mm | −2.16° (1.89), 24.8 mm | −2.89° (1.29), 30.8 mm |
| [02](squares/2026-11-25-run02-turn-aware-ekf-and-ramps.csv) | Our EKF learns the gyro's offset only while not turning; `odom_drive.py` ramps its speed | 68 mm, 4.2° | −1.28° (1.24), 12.5 mm | −2.05° (1.14), 27.3 mm | −3.03° (1.30), 44.1 mm |
| [03](squares/2026-11-26-run03-steered-by-ekf-on-the-pi.csv) | Our EKF publishes `odom → base_link`; `odom_drive.py` steers by it, on the Pi, and stops 11 mm and 1.9° short | 2.7 mm, 0.12° | +0.75° (0.09), 21.2 mm | −0.92° (2.62), 33.2 mm | −1.35° (1.57), 29.1 mm |
| [04](squares/2026-11-26-run04-gyro-filter-10hz.csv) | Gyro filter 10 Hz instead of 44 Hz; stops 14 mm and 2.8° short | 1.5 mm, 0.08° | +0.17° (0.05), 7.9 mm | −1.04° (1.49), 24.9 mm | −1.52° (1.39), 28.2 mm |
| [05](squares/2026-11-27-run05-gyro-recheck.csv) | Nothing: a recheck on another day, one square each way | 1.8 mm, 0.16° | +0.44° (0.16), 8.0 mm | −0.80° (1.38), 15.5 mm | −2.48° (0.91), 33.9 mm |

Each estimate's mean turn error per square, positive when it counted more turn than the robot made, with its spread (sample standard deviation) over the session's squares; then the mean distance between where it said the robot ended and where it did.

- **Run 01:** the recording's first three left squares (drives 0–2) were measured less carefully and aren't listed; drive 8 was stopped part-way. Our EKF followed the wheels: with the gyro's offset in its state, it put the gyro's extra turn down to offset.
- **Run 02:** drive 4 hit something and isn't listed.
- **Run 03:** recorded on the laptop, whose clock ran 1.5 s ahead of the Pi's; the tool times each command by the `/odom` stamp recorded just before it, so this doesn't matter. Drive 0 was the same square steered from the laptop: one turn stopped 3.2° past its target and our EKF ended 51 mm off, so `odom_drive.py` now runs on the Pi. The robot's near-perfect end was two errors cancelling: our EKF counted 0.75° too much turn per square, and the turns carried on about as far past their targets.
- **Run 04:** also two turns on the spot each way (`spin 720`, `spin -720`). Our EKF counted 0.02% too much turn, robot_localization 0.42% too little and `/odom` 0.74% too little.
- **Run 05:** the day after run 04, before the LiDAR went on, to check the gyro's error no longer changes between days. Two turns on the spot each way came first (drives 0–1). The Pi had been turned and moved to the robot's right side since run 04; the IMU hadn't moved. Two turns on the spot: our EKF 0.027% too much, robot_localization 0.34% and `/odom` 0.65% too little. The right square's turns ran 0.66–1.05° past their targets, against about 0.25° in run 04, and our EKF's turn error was larger there: 0.55°, against 0.32° in the left square. Battery 11.85–11.71 V, against 12.09–11.82 V in run 04.

What the runs show:

- **`/odom` misses some of every turn.** The body turns further than the wheels count, mostly as it starts and stops, probably from the gearbox's backlash and the tyres' give, since the encoders are on the motor side. The gyro sees it; the wheels can't.
- **The 44 Hz gyro filter let vibration alias.** The gyro is read at 50 Hz, so anything between 25 and 44 Hz, mostly motor vibration, folded into false slow readings, and the gyro's error changed from session to session: our EKF, which mostly follows the gyro, read 1.3° short per square in run 02 and 0.75° over in run 03. With the 10 Hz filter the gyro's spread while driving straight fell from 2.0 × 10⁻² to 1.2 × 10⁻² rad/s (variance 3.9 × 10⁻⁴ to 1.3 × 10⁻⁴, from `tools/odom_imu_noise.py` on runs 03 and 04), and our EKF's error from 0.75° to 0.17° per square. A day later (run 05) two turns on the spot read the same as in run 04, 0.027% against 0.02%, so the gyro's error now holds between days.
- **How far the robot carries on.** By our EKF, with `odom_drive.py` on the Pi, run 03's turns ended 0.89° past their targets (spread 0.18) and its sides 3.1 mm past (spread 1.5), stopping 1.9° and 11 mm short; run 04's turns ended 0.24° past and its sides 0.1 mm short, stopping 2.8° and 14 mm short. The extra 0.24° is the 10 Hz filter's 9 ms delay at 0.5 rad/s, so `odom_drive.py` now stops 3.0° and 14 mm short. In run 05 the turns ended 0.48° past (spread 0.37) and the sides 0.3 mm past.
- **`/odom` now misses 0.65–0.74% of two turns on the spot** (runs 04 and 05), against 0.2% after the floor calibration on 25 November. It isn't explained yet, and it doesn't affect our EKF.


## Maps ([maps/](maps/))

Maps from SLAM Toolbox (`ros2 launch robot_description slam.launch.py`, with `ros/robot_description/config/slam.yaml`), built on the Pi while driving by teleop at 0.1 m/s and 0.5 rad/s, and saved from the laptop with `ros2 run nav2_map_server map_saver_cli -f <name>`. Each map is a `.pgm` image with its `.yaml` (5 cm cells, and where the image sits in the `map` frame), plus a `.png` marking the tape checks, which GitHub can show.

Distances in the room were measured with a tape at about the laser's height, 15 cm up, and are approximate rather than exact to the centimetre. On the map, a wall's position is the average column of its occupied cells over the rows listed in the `-tape.csv`, and a distance is the gap between two walls; a box's size is the distance between the centres of its outermost cells.

Uncertainties are one standard deviation. A wall can be anywhere in its 5 cm cell, so its position on the map is uncertain by ±14 mm (5 cm / √12), and a distance between two walls or edges by ±20 mm; ±50 mm at the very worst. The tape readings are taken as ±10 mm, as they were approximate. A difference combines the two: ±22 mm.

| Run | What | Tape against map |
| --- | --- | --- |
| [01](maps/2026-11-27-run01-first-map.png) | First map: one room, driven round once and back to the start | Five distances, 0.15–2.9 m: −36 to +8 mm, 12 mm on average, each ±22 mm |

Run 01's distances ([2026-11-27-run01-first-map-tape.csv](maps/2026-11-27-run01-first-map-tape.csv)):

| Distance | Tape | Map | Difference |
| --- | --- | --- | --- |
| Top part: middle wall to the cabinet's front | 2.901 ± 0.010 m (114.2 in) | 2.894 ± 0.020 m | −7 ± 22 mm |
| Lower part: width at its upper end | 1.235 ± 0.010 m, re-measured (first read as 49.25 in, 1.251 m) | 1.225 ± 0.020 m | −10 ± 22 mm |
| Lower part: width at its lower end | 1.121 ± 0.010 m (44 1/8 in) | 1.129 ± 0.020 m | +8 ± 22 mm |
| Box: length | 0.686 ± 0.010 m (27 in) | 0.650 ± 0.020 m, or more: its far end isn't mapped | −36 ± 22 mm |
| Box: width | 0.149 ± 0.010 m (5 7/8 in) | 0.150 ± 0.020 m | +1 ± 22 mm |

Four of the five differences are well inside their ±22 mm. The fifth, the box's length, is 1.6 times it, and there the map can only say the box is at least that long.

- **Run 01:** 113 × 195 cells (5.65 × 9.75 m). The long strip at the bottom is a hallway, seen through a doorway. The box is the one used to check which way the LiDAR faces, beside where the robot started. Its far end isn't marked as wall, only its two sides reach it, so its length is the least certain distance. The lower part narrows by 11 cm between the two tape lines; the map has it narrowing by 10 cm. The robot was also steered round the room by watching only the map in RViz, passing close to objects.
