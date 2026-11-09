# Bench Tests

Step-by-step checks for each part, with the results recorded as they are run. Tests 1–5 and the battery check passed by 6 November 2026, ahead of the return windows, which close around 17–23 November. Soldering and modifying parts is now fine.

## Status

| Test | Status | Date | Result |
| --- | --- | --- | --- |
| [Batteries](#batteries) | Pass | 2026-11-05 | 3.82 / 3.83 / 3.83 V; no damage |
| [1. Pi power](#1-pi-power) | Pass | 2026-11-06 | Lowest 5.16 V, `throttled=0x0`, peak 54.9 °C |
| [2. LiDAR](#2-lidar) | Pass | 2026-11-05 | Health OK, 10 Hz, live scan in RViz |
| [3. Encoders at 3.3 V](#3-encoders-at-33-v) | Pass, both motors | 2026-11-05 | Works at 3.3 V; no 5 V divider needed |
| [4. Spin and stall current](#4-spin-and-stall-current) | Pass, both motors | 2026-11-06 | Stall about 2.4–3.2 A; free spin about 0.15 A, slightly above expected |
| [5. Odometry constants](#5-odometry-constants) | Pass, both motors | 2026-11-06 | About 68 mm wheels; about 2473 ticks per turn |
| [6. Bumper and low-battery cutoff](#6-bumper-and-low-battery-cutoff) | Pass | 2026-11-09 | Bumpers and low-battery cutoff stop the motors; stall check also confirmed |
| [7. MPU-6050](#7-mpu-6050) | Pass | 2026-11-06 | Found at `0x68`; Z about 15,470 flat, needs calibrating |

## What's left

Nothing: every bench test has passed. The DEN's bench supply is still useful for teleop tests before the robot runs on its battery; return it, the leads and the calipers by the loan's due date.

## Results to carry forward

- **Encoders:** they run at 3.3 V, so no 5 V divider is needed. Keep the Pico's internal pull-ups on, as the test script does.
- **Odometry starting values:** wheel diameter about 68 mm, about 2473 ticks per wheel turn.
- **IMU calibration:** lying flat, the MPU-6050 read about X 1250, Y 150 and Z 15,470, so calibrate its offsets before using it in the EKF.
- **Safety thresholds (firmware):** low-battery stop at 10.5 V, Pi shutdown warning at 10.2 V, re-arm above 11.0 V; stall stop at power ≥ 0.5 and speed < 0.05 rev/s for 0.5 s.
- **Stall protection:** the estimated stall current of about 2.4–3.2 A can exceed the MDD3A's 3 A continuous rating, so the firmware must not leave a motor stalled.

## Batteries

**Needs:** both packs, a voltage alarm, and a clear, non-flammable surface. Keep the charger unplugged.

1. Inspect each pack for puffing, dents, tears, damaged wires or a sweet or chemical smell. If anything looks wrong, stop, and move the pack away from anything flammable.
2. Leave the tight plastic wrap on. It's the pack's outer protective layer, not packaging. Never cut it or use a blade near it.
3. Find the balance plug: the small white connector with 4 thin wires. Keep metal away from the XT60's contacts.
4. Line up the plug's black wire with the alarm's `-` pin and press it on gently. The plug covers 4 of the pins. Don't force it or guess the polarity.
5. Read the total voltage and each cell's voltage. A loud continuous beep could mean a low cell or a plug on wrong, so unplug it and investigate.
6. Unplug the alarm, because it slowly drains the pack. Repeat for the second pack.

**Pass:** each cell reads 3.7–3.9 V, with all three within about 0.05 V of each other. **Return the pack** if any cell is under 3.0 V or far from the others.

Don't charge the packs until they're first needed. Do the first charge inside the LiPo bag, on a hard surface, and stay nearby.

### Results

| Pack | Total | Cell 1 | Cell 2 | Cell 3 | Inspection |
| --- | --- | --- | --- | --- | --- |
| 1 | 11.5 V | 3.82 V | 3.83 V | 3.83 V | No puffing, damage or smell |
| 2 | 11.5 V | 3.82 V | 3.83 V | 3.83 V | No puffing, damage or smell |

## 1. Pi power

**Needs:** Pi 5, active cooler, microSD card, USB-C power supply and laptop; then the DEN bench supply and the converter.

### Set up the Pi

1. In Windows, not WSL, open [Raspberry Pi Imager](https://www.raspberrypi.com/software/). Choose Raspberry Pi 5, then Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)**. Don't use 26.04, which ROS 2 Jazzy doesn't support. If Imager doesn't list 24.04, the image is on [cdimage.ubuntu.com](https://cdimage.ubuntu.com/releases/24.04/release/).
2. In the customisation step, set the hostname (e.g. `robot-pi`), username, password, WiFi with country `CA`, and SSH with password login. Use home WiFi or a phone hotspot, because this setup probably won't handle eduroam's login. The laptop must be on the same network.
3. With the Pi unplugged, fit the active cooler, plug its cable into `FAN`, and insert the card.
4. Boot from the USB-C power supply and wait 3–5 minutes. Connect from Windows PowerShell:
   ```
   ssh yourusername@robot-pi.local
   ```
5. Check the install. Expect Ubuntu 24.04.x and `aarch64`:
   ```bash
   lsb_release -d && uname -m
   ```
6. Update the system and install the test tool:
   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo apt install stress-ng
   ```
7. Add `usb_max_current_enable=1` to `/boot/firmware/config.txt`, then reboot.

### Load test the converter

1. Set the bench supply to 12 V, clip it to the converter's input wires without soldering, and plug the converter into the Pi.
2. Run the load:
   ```bash
   stress-ng --cpu 4 --timeout 120s
   ```
3. Meanwhile, in a second SSH session, log the readings. Start this first, since it runs slightly longer than the load. If `vcgencmd` is missing, install `libraspberrypi-bin`, or `raspi-utils` if that package doesn't exist.
   ```bash
   for i in $(seq 1 14); do sudo vcgencmd get_throttled; sudo vcgencmd pmic_read_adc EXT5V_V; sudo vcgencmd measure_temp; sleep 10; done
   ```

**Pass:** `throttled=0x0` throughout, and the 5 V reading stays at 4.8 V or above. If it fails, the Pololu D36V50F5 is the fallback converter.

### Results

- **Setup, 2026-11-05:** first boot and SSH over WiFi work; Ubuntu 24.04.5 on `aarch64`. On the USB-C charger the Pi warns that the supply can't provide 5 A, which is expected. System updated, `usb_max_current_enable=1` set, and `stress-ng` and `vcgencmd` both installed. Stay on 24.04 and ignore the login banner's offer of 26.04, which ROS 2 Jazzy doesn't support.
- **Converter load test, 2026-11-06:** Pass on a 12 V bench supply, with a CPU load only and no USB devices attached. `throttled=0x0` throughout, `EXT5V_V` stayed at 5.16–5.22 V, temperature peaked at 54.9 °C, and the fan came on. The supply's peak current wasn't recorded.

## 2. LiDAR

**Needs:** RPLIDAR C1, USB-C adapter and laptop.

1. Build the driver once, in an Ubuntu terminal. It sits next to this repo in the workspace, not inside it:
   ```bash
   cd ~/robot_ws/src
   git clone --depth 1 https://github.com/Slamtec/sllidar_ros2.git
   cd ~/robot_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select sllidar_ros2 --symlink-install
   ```
2. WSL2 doesn't see USB devices by default, so pass the adapter through with [usbipd-win](https://github.com/dorssel/usbipd-win). Install it once in Windows PowerShell with `winget install --exact dorssel.usbipd-win`.
3. Plug in the LiDAR, then run these in PowerShell as Administrator, with an Ubuntu terminal open. Use the bus ID that `usbipd list` shows for the CP2102N. It was `3-1` here, and it can change with the port. `bind` is only needed once; `attach` is needed each time it's reconnected.
   ```
   usbipd list
   usbipd bind --busid 3-1
   usbipd attach --wsl --busid 3-1
   ```
4. In the Ubuntu terminal, start the driver and RViz. The `chmod` is needed after each attach, and the `source` in each new terminal.
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   source ~/robot_ws/install/setup.bash
   ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
   ```

RViz's `wrong permissions on runtime directory` warning is harmless under WSL.

Alternatively, plug the LiDAR into the Pi and view the scan from the laptop once the Pi has ROS 2.

**Pass:** the driver reports health OK, and a live scan in RViz shows the room and reacts to a hand near the LiDAR.

### Results

- **2026-11-05:** Pass. The driver reported health OK, Standard scan mode at 10 Hz and firmware 1.02. RViz showed the room and reacted to a hand.

## 3. Encoders at 3.3 V

**Needs:** one motor, Pico and breadboard.

1. Power one encoder from the Pico's 3V3(OUT) pin, for this test only.
2. Connect C1 and C2 to GP2 and GP3. Leave M1 and M2 unconnected.
3. Count edges in MicroPython while turning the wheel by hand, using this script in Thonny:

   ```python
   from machine import Pin
   import time

   a = Pin(2, Pin.IN, Pin.PULL_UP)
   b = Pin(3, Pin.IN, Pin.PULL_UP)

   STEP = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]
   count = 0
   skipped = 0
   state = (a.value() << 1) | b.value()

   def on_edge(pin):
       global count, skipped, state
       new = (a.value() << 1) | b.value()
       index = (state << 2) | new
       if index in (3, 6, 9, 12):
           skipped += 1
       count += STEP[index]
       state = new

   a.irq(on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)
   b.irq(on_edge, Pin.IRQ_RISING | Pin.IRQ_FALLING)

   last = None
   print("Turn the wheel by hand. Press Stop to end.")
   print("Start: C1 =", a.value(), " C2 =", b.value())
   while True:
       if count != last:
           last = count
           print("count:", count, " skipped:", skipped)
       time.sleep(0.2)
   ```

**Pass:** the count goes up one way and down the other, and one full turn is roughly 2464.

**If it stays at 0,** first check for 3.3 V across the motor's VCC and GND wires, and look for loose breadboard connections. If those are fine, the encoder may need 5 V. Power it from VBUS with C1 disconnected from the Pico, and measure C1 with the multimeter while turning. If it swings to about 5 V, use the 10k/20k divider.

### Results

| Motor | Date | Result |
| --- | --- | --- |
| First | 2026-11-05 | Pass at 3.3 V: the count moved both ways, reached −2334 on one hand-judged turn, and skipped 0. The script uses the Pico's internal pull-ups. |
| Second | 2026-11-05 | Pass at 3.3 V, as reported at the time; exact counts not recorded. |

## 4. Spin and stall current

**Needs:** both motors and the DEN bench supply. Clip the supply to M1 and M2.

1. At 12 V with a 0.5 A limit, let the motor spin.
2. At 3 V with about a 1.5 A limit, hold the wheel still for under 2 seconds and read the current. Multiply by 4 to get the 12 V stall current.

**Pass:** it spins smoothly at about 0.12 A or less, and the calculated stall current is 5 A or less. **If stall is over 5 A,** limit the duty cycle in firmware or move to the Cytron MDD10A.

### Results

| Motor | Free-running current at 12 V | Current held still at 3 V | Calculated stall at 12 V |
| --- | --- | --- | --- |
| Both, approximate | About 0.15 A | About 0.6–0.8 A | About 2.4–3.2 A |

Both motors read about the same and spun normally. Free spin is a little above the expected 0.12 A. The top of the stall estimate is above the MDD3A's 3 A continuous rating but within its 5 A peak, so the firmware must not leave a motor stalled.

## 5. Odometry constants

**Needs:** both motors, the Pico setup from test 3, and DEN calipers.

1. Measure the wheel diameter with calipers. The listing doesn't state it; about 65 mm is expected.
2. Count encoder ticks over 10 marked wheel turns. About 2464 per turn is expected: 11 pulses × 4 edges × 56:1 gearbox.

### Results

| Motor | Wheel diameter | Ticks over 10 turns | Ticks per turn |
| --- | --- | --- | --- |
| First | About 68 mm | 24,734, with 0 skipped | About 2473 |
| Second | About 68 mm | Reported the same as the first | About 2473 |

The wheels are larger than the 65 mm estimate, and the tick count is within 0.4% of the expected 2464. Treat these as starting values for odometry, to be refined with the square-path runs in milestone 2.

## 6. Bumper and low-battery cutoff

**Needs:** working firmware, the MDD3A, bumper switches and the bench supply. Lift the wheels off the bench.

1. Press each bumper switch while the wheels spin.
2. Run the battery input from the bench supply and lower it past 10.5 V (3.5 V per cell).

**Pass:** the motors stop both times.

### Results

- **2026-11-09, pass,** with the firmware's safety test mode (wheels lifted, 12 V bench supply with a 1.5 A limit; battery read through the 33k/10k divider on GP28, from the same supply). Logs and details in [results/README.md](../results/README.md#safety-checks-safety).
- **Bumpers:** every press stopped both motors at once, and they restarted only after the next commanded stop.
- **Low battery:** lowering the supply to 10.3 V while the wheels ran stopped them as the filtered reading crossed 10.5 V; it cleared once the supply was back above 11.0 V.
- **Stall:** on the bench, holding a wheel stopped it within about 0.3 s, but through the low-battery cutoff: the 1.5 A limit made the supply's voltage sag faster than the 0.5 s stall check. Unplugging an encoder's ground mid-run (speed reads 0 while power is high) did trigger the stall check. On a battery, with no current limit, the stall check is the one that acts.

## 7. MPU-6050

**Needs:** MPU-6050, Pico and soldering iron. Its header pins are the one thing that's fine to solder early.

1. Solder the header pins.
2. Wire VIN to the Pico's 3V3(OUT), GND to GND, SDA to GP4 and SCL to GP5.
3. Run an I2C scan, then read the acceleration with the board lying flat and its printed side up, using this script in Thonny:

   ```python
   from machine import I2C, Pin
   import time

   i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
   print("Found:", [hex(d) for d in i2c.scan()])

   ADDRESS = 0x68
   i2c.writeto_mem(ADDRESS, 0x6B, b"\x00")
   time.sleep(0.1)

   def signed(high, low):
       value = (high << 8) | low
       return value - 65536 if value >= 32768 else value

   while True:
       d = i2c.readfrom_mem(ADDRESS, 0x3B, 6)
       print("X:", signed(d[0], d[1]), " Y:", signed(d[2], d[3]), " Z:", signed(d[4], d[5]))
       time.sleep(0.5)
   ```

**Pass:** the scan finds address `0x68`, and Z reads about 16384 (1 g), with X and Y much smaller. Some offset is normal.

### Results

- **2026-11-06:** Pass, with the Adafruit board's own pull-ups only. Found at `0x68`. Lying flat, it read about X 1250, Y 150 and Z 15,470, so about 0.94 g on Z. Not yet calibrated.
