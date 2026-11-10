# Results

Test data, logs and charts from the robot. Bench-test procedures and pass/fail results are in [docs/bench-tests.md](../docs/bench-tests.md); this folder holds the raw data behind firmware tests.

## Naming

`YYYY-MM-DD-runNN-<load>-<speed>-<what changed>`, with the CSV and its chart sharing a name. Run numbers count up within each folder and are zero-padded so they sort in order. `<load>` is `no-load` (wheels lifted) or `fingers-on` (light finger pressure on both wheels); `<speed>` is the test target in revolutions per second; `<what changed>` names the change since the previous run.

CSVs are kept exactly as the firmware logged them, header included.

## PID speed loop ([pid/](pid/))

Logged at 50 Hz by [firmware/src/main.cpp](../firmware/src/main.cpp). Columns: `t_ms` (since the test started), `target` (rev/s, see below), then each wheel's measured speed (rev/s, the average over the last 20 ms) and motor power (PWM fraction, -0.95 to +0.95). Plot with:

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

Keyboard driving through `pico_bridge` (`ros/pico_bridge`), recorded with `ros2 bag record /cmd_vel /joint_states /battery /rosout` into `~/robot_ws/bags/` (outside the repo). Convert with `python3 tools/plot_teleop.py <bag dir> <output.csv> <output.png> "<title>"` (ROS 2 sourced). Columns: `t_ms` (counted from the first wheel reading, 20 ms per Pico status), each wheel's commanded and measured speed (rev/s; commands rebuilt from `/cmd_vel` with the bridge's 1 s timeout), and `battery_volts`. Left wheel is the first motor, neither reversed. Every run: wheels lifted, 12 V bench supply with a 1.5 A limit, keyboard at 0.1 m/s and 0.5 rad/s.

**WSL's clock steps backwards.** Inside WSL the system clock runs fast and is pulled back to Windows time about every 30 s, by 0.6–2.4 s. Every ROS timestamp on the laptop jumps back with it, so a recording read in timestamp order interleaves the next second or two with the previous one: in the first charts this looked like the wheels flicking between two speeds. The data itself was always in order (a live subscriber saw no reordering, and a fake Pico on a pseudo-terminal reproduced it with no hardware). `plot_teleop.py` therefore reads recordings in file order and counts time from the Pico's 20 ms status rate. Disabling Ubuntu's own time sync (`systemd-timesyncd`) roughly halved the jumps but didn't remove them. The Pi runs native Ubuntu, so this only affects the laptop.

| Run | Driving | Result |
| --- | --- | --- |
| [01](teleop/2026-11-09-run01-no-load-keyboard.png) ([CSV](teleop/2026-11-09-run01-no-load-keyboard.csv)) | Forward, backward, turns on the spot and arcs | Both wheels followed every command. |
| [02](teleop/2026-11-09-run02-no-load-keyboard-and-bridge-stop.png) ([CSV](teleop/2026-11-09-run02-no-load-keyboard-and-bridge-stop.csv)) | As run 01, then the bridge stopped mid-drive | Both wheels followed the commands. Stopping the bridge (Ctrl+C) mid-drive stopped the wheels through the Pico's 250 ms command timeout, as seen on the bench; the recording ends with the bridge, so it isn't in the data. |
| [03](teleop/2026-11-09-run03-no-load-keyboard-and-bumper.png) ([CSV](teleop/2026-11-09-run03-no-load-keyboard-and-bumper.csv)) | Driving with two bumper presses | Each press stopped both wheels while the command continued; the bridge logged the stop and the clear, and driving resumed. |
| [04](teleop/2026-11-09-run04-no-load-keyboard-quick-switching.png) ([CSV](teleop/2026-11-09-run04-no-load-keyboard-quick-switching.csv)) | 60 s with quick key switching | Both wheels followed every change. |
