# Parts List — Budget Build

The robot's parts cost about CAD $645, and your own soldering iron and multimeter add about $61, for **about CAD $706 before tax and shipping**. The parts alone are down from the build plan's original estimate of about $930. Raspberry Pi prices rose roughly 40% during 2026, so the Pi 5 4GB alone is now $155. The parts are the cheapest set that still meets two conditions:

- linorobot2's firmware supports every part, so it remains the fallback.
- The encoders and IMU are good enough for the odometry and EKF in milestone 2.

Prices and stock were checked on 17 October 2026. The parts were ordered that day and had arrived by 28 October (see [Order status](#order-status)). The [bench tests](#bench-tests) haven't been run yet.

Every part that goes on the robot is bought, not borrowed, so the robot can stay assembled. You also buy the two tools you'll use every week, a soldering iron and a multimeter. Only the bench power supply, oscilloscope and calipers are borrowed from the [DEN](https://den.uwaterloo.ca/inventory/search), for a few one-off tests. Your SYDE 192L kit already covers the breadboard, jumper wires, wire cutters, buttons and LEDs.

## Shopping list

### CanaKit

The site shows US dollars by default; switch the currency to CAD before checking out.

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Raspberry Pi 5, 4 GB | 154.95 | [canakit.com](https://www.canakit.com/raspberry-pi-5-4gb.html) | Onboard computer: Ubuntu 24.04, ROS 2 Jazzy, SLAM, navigation |
| Raspberry Pi 5 Active Cooler | 6.95 | [canakit.com](https://www.canakit.com/raspberry-pi-5-active-cooler.html) | Prevents the Pi from slowing down when it overheats under SLAM load |

### PiShop.ca

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Raspberry Pi Pico 2 WH (headers already soldered) | 11.95 | [pishop.ca](https://www.pishop.ca/product/raspberry-pi-pico-2w-with-header/) | Motor microcontroller: encoders, wheel PID, watchdog. linorobot2 env `pico2w`. |
| Adafruit MPU-6050 (#3886) | 18.85 | [pishop.ca](https://www.pishop.ca/product/adafruit-mpu-6050-6-dof-accel-and-gyro-sensor-stemma-qt-qwiic/) | IMU for the EKF. Genuine chip. linorobot2 `USE_MPU6050_IMU`. |
| SanDisk Extreme Pro 32 GB microSD (A1) | 29.95 | [pishop.ca](https://www.pishop.ca/product/sandisk-extreme-pro-microsdxc-card-32-gb-class-10-blank/) | Pi operating system |
| Micro-HDMI to HDMI cable, 3 ft | 5.95 | [pishop.ca](https://www.pishop.ca/product/micro-hdmi-to-hdmi-cable-for-pi-4-3ft-black/) | Lets you plug in a screen if the Pi doesn't come up on WiFi at first boot |
| USB-A to micro-USB cable, 45 cm | 2.95 | [pishop.ca](https://www.pishop.ca/product/usb-cable-a-to-micro-b-45cm-black/) | Connects the Pi to the Pico (data, not charge-only). Also flashes the Pico from the laptop through your USB adapter. |

PiShop charges shipping by weight, shown in the cart. Local pickup in Ottawa is free.

### Motor driver: DigiKey.ca or RobotShop

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Cytron MDD3A dual motor driver | 15.81 | [digikey.ca](https://www.digikey.ca/en/products/result?keywords=1597-105090004-ND) | 4–16 V; 3 A continuous / 5 A peak per channel; 3.3 V logic; reverse-polarity protection |

- **DigiKey:** listed under Seeed (part 105090004), but it is the genuine Cytron board. Shipping is $15 on orders under $100, so about $31 in total.
- **RobotShop CA:** [about $12.50](https://ca.robotshop.com/products/cytron-3a-4-16v-dual-channel-dc-motor-driver), from a search snippet only. Compare the two totals at checkout.
- **PiShop** doesn't carry it.

### Amazon.ca

| Part | Qty | Price | Link | Used for |
| --- | --- | --- | --- | --- |
| RPLIDAR C1 (Waveshare) | 1 | 93.99 | [B0CT31PH8S](https://www.amazon.ca/dp/B0CT31PH8S) | 2D LiDAR: time-of-flight, 12 m range, 10 Hz. Includes a USB adapter and a USB-A to USB-C cable. linorobot2 `--laser c1`. |
| JGB37-520 encoder motor set (choose **178 RPM**) | 2 | 38.66 (19.33 each) | [B0H4RH7MRN](https://www.amazon.ca/dp/B0H4RH7MRN) | Drive motors: 12 V, 56:1 gearbox, 6 mm D-shaft. Each set includes a wheel, bracket, coupling and screws. |
| Zeee 3S 2200 mAh 50C LiPo, XT60, 2-pack | 1 | 42.29 | [B0D5LKXHBK](https://www.amazon.ca/dp/B0D5LKXHBK) | Main battery. The second pack charges while the first runs. $46.99 without Prime. |
| Klnuoxj 12/24 V to 5 V 5 A USB-C converter | 1 | 27.78 | [B0CRVW7N2J](https://www.amazon.ca/dp/B0CRVW7N2J) | Powers the Pi 5. 8–32 V input; USB-C plug; overload, overcurrent and overheat protection. 4.5★ from 123 reviews, including Pi 5 users. |
| Inline blade-fuse holders, 3-pack | 1 | 9.99 | [B0D47LJHTW](https://www.amazon.ca/dp/B0D47LJHTW) | Main fuse at the battery. Comes with 20 A and 30 A fuses only; don't use those. |
| Blade fuse assortment, 2–10 A | 1 | 7.99 | [B09H1ZN75M](https://www.amazon.ca/dp/B09H1ZN75M) | Use a 10 A fuse in the main holder |
| DaierTek mini rocker switches, 10-pack | 1 | 11.99 | [B07S2QJKTX](https://www.amazon.ca/dp/B07S2QJKTX) | Physical motor-power cutoff. Rated 20 A at 12 V DC. |
| XT60 connectors, 10 pairs | 1 | 10.99 | [B07233RVL6](https://www.amazon.ca/dp/B07233RVL6) | Battery connector on the robot side |
| KangTeer ball transfer units, 5/8" ball, 5 pcs | 1 | 11.89 | [B072M9S2CQ](https://www.amazon.ca/dp/B072M9S2CQ) | Caster (third wheel); 15 kg rating. The small ball suits hard floors and low carpet, and can catch on thick carpet or door thresholds. |
| Nylon standoff kit, M2 / M2.5 / M3 | 1 | 12.99 | [B07WSCZ2KH](https://www.amazon.ca/dp/B07WSCZ2KH) | Mounting the boards (see the mounting check below) |
| HiLetgo KW12-3 roller-lever microswitches, 10-pack | 1 | 20.00 | [B07X142VGC](https://www.amazon.ca/dp/B07X142VGC) | Front bumper: two switches behind a 3D-printed bumper, wired to the Pico, so the motors stop on contact even if the Pi or ROS has crashed. Catches obstacles below or above the LiDAR's scan plane. Each switch is 20 × 10.5 × 6.5 mm. |
| HTRC 2–3S balance charger | 1 | 23.99 | [B0H4PLBZ84](https://www.amazon.ca/dp/B0H4PLBZ84) | LiPo charging. Plugs straight into the wall (100–240 V AC). |
| LiPo safe bags, 2 pcs | 1 | 15.50 | [B0F6LP63HD](https://www.amazon.ca/dp/B0F6LP63HD) | Charging and storing the batteries. Both packs fit in one bag. |
| LiPo low-voltage alarms, 2-pack | 1 | 12.96 | [B073WXVRBT](https://www.amazon.ca/dp/B073WXVRBT) | Beeps before a cell drops low enough to be damaged (default 3.3 V per cell). Use it until your firmware monitors the battery, and unplug it before storing a pack. |
| Plusivo 60 W digital soldering iron kit | 1 | 36.30 | [B07S61WT16](https://www.amazon.ca/dp/B07S61WT16) | Your own iron: adjustable 200–500 °C, 5 tips, solder, stand, desoldering pump. 4.7★ (225). |
| AstroAI 132A multimeter | 1 | 24.29 | [B07FDBW1PT](https://www.amazon.ca/dp/B07FDBW1PT) | Auto-ranging; measures DC current (needed for the stall test), voltage, continuity and capacitance. 4.5★ (8,854). |

For the motors, the RPM option is the only choice on the listing, and 178 RPM is what this plan assumes. Order a quantity of 2.

The LiPo bag's price depends on the seller: $15.50 was the cheapest offer on 17 October, and another seller charges $17.99 for the same bags.

### Anywhere (estimates)

Check the DEN's free items and RidgidWare (PSE 1419) before buying these. Free DEN items are yours to keep.

| Part | Approx. price | Used for |
| --- | --- | --- |
| Deck: 3D-printed plate, or 1/4" plywood | 10 | Chassis. Also print the motor, LiDAR and caster mounts, and a spacer to level the caster. |
| Wiring and small parts:<br>• perfboard<br>• 16–18 AWG silicone wire<br>• resistor assortment<br>• electrolytic capacitor assortment (220–470 µF)<br>• AMS1117-3.3 regulator modules<br>• screw terminals<br>• heat shrink | 47 | The final wiring. A breadboard shakes loose on a moving robot, so use perfboard. The motor cables end in bare wire, so solder them or clamp them in screw terminals. |

## Budget

| Group | CAD |
| --- | --- |
| Compute: Pi 5 4 GB, cooler, microSD, micro-HDMI cable | 197.80 |
| LiDAR | 93.99 |
| Motors and wheels | 38.66 |
| Microcontroller, driver, IMU, Pico USB cable | 49.56 |
| Battery and power: batteries, converter, fuse holders, fuses, switch, connectors | 111.03 |
| Chassis and bumper: casters, standoffs, deck, microswitches | 54.88 |
| Wiring and small parts | 47.00 |
| Charging and safety: charger, LiPo bags, alarms | 52.45 |
| Tools you keep: soldering iron kit, multimeter | 60.59 |
| **Total before tax and shipping** | **705.96** |

About $57 of that is estimates: the deck and the small parts. Shipping, import fees and sales tax aren't included.

## Order status

Everything except the small parts was ordered on 17 October 2026 and had arrived by 28 October, ahead of the delivery estimates.

- [x] **Amazon.ca:** the 16 Amazon listings, $401.60.
- [x] **CanaKit:** Pi 5 and cooler, $161.90.
- [x] **PiShop:** Pico 2 WH, MPU-6050, microSD card, micro-HDMI cable and micro-USB cable, $69.65.
- [x] **MDD3A:** from RobotShop or DigiKey.
- [ ] **Small parts:** check the DEN's free items and RidgidWare, then buy whatever is left.
- [ ] **Borrow tools from the [DEN](https://den.uwaterloo.ca/inventory/search)** (free for SYDE and BME undergrads; book a pickup with Chris McClellan, E7-6414):
  - digital caliper
  - 2231A-30-3 bench power supply (0–30 V, 3 A), for bench tests 1, 4 and 6. Its fine if not returned is $1,000.
  - oscilloscope, for bench test 3 (optional)
  - Return them when the build is done.

## Fallbacks and optional upgrades

These aren't part of the build. Use them only if a bench test fails, or if you decide to spend more.

| If… | Then |
| --- | --- |
| The converter shows undervoltage in bench test 1 | Pololu D36V50F5, about $60–70 with shipping. It's a bare board, so it needs a USB-C pigtail or wiring to the Pi's GPIO power pins. |
| The stall current is over 5 A in bench test 4 | Limit the duty cycle in firmware, or move to the Cytron MDD10A, about $30 |
| You want room for recorded data | [SanDisk Ultra 128 GB A1](https://www.pishop.ca/product/microsd-card-ultra-128gb-class-10-a1-blank/) instead of the 32 GB card, +$10 |
| You want on-chip sensor fusion | Adafruit BNO085 instead of the MPU-6050, +$25. Wire it to the Pico; the Pi's I2C has timing trouble with it. |
| You want more memory | Pi 5 8 GB, +$90 |

## Compatibility checks

| Check | Parts | Result |
| --- | --- | --- |
| Motor voltage vs battery | 12 V motors, 3S LiPo (11.1 V nominal, 12.6 V full) | OK. A full charge is 5% over the motor rating, so cap the PWM duty at about 95% (12 / 12.6) in firmware. |
| Driver voltage | MDD3A (16 V max) vs 12.6 V | OK. The DRV8833 (10.8 V max) would not be. |
| Driver current vs motor stall | MDD3A 3 A continuous, 5 A peak | Probably OK. The motor listing gives no stall current; other sellers of this motor list 2.3–2.4 A. Measure it (see below). |
| Driver inputs | MDD3A datasheet | OK. Two PWM inputs per motor: A high and B low runs forward, A low and B high runs backward, both low or both high brakes. Logic high is anything from 1.7 V, so 3.3 V works. |
| Encoder signals vs 3.3 V logic | Encoder supply voltage not stated on the listing; Pico pins take 3.3 V only | Power the encoders at 3.3 V so their outputs are 3.3 V. If they need 5 V, put a 10k/20k divider on each A/B line. |
| Pi 5 supply | Klnuoxj 5 V 5 A converter; RPLIDAR C1 draws 230 mA running and **800 mA at startup** | The converter doesn't negotiate USB-PD, so the Pi limits its USB ports to 600 mA total, which is below the LiDAR's startup draw. Add `usb_max_current_enable=1` to `/boot/firmware/config.txt`; the converter is rated 5 A. Its output is 5.0 V rather than the official supply's 5.1 V, so check for undervoltage with `vcgencmd get_throttled`. If that shows undervoltage, remove the setting and move to a Pololu D36V50F5 (5.5 A, about $60–70 with shipping). |
| Mounting | Pi 5: M2.5 holes<br>RPLIDAR C1: M2.5 threaded holes, screws no deeper than 4 mm<br>MDD3A: 3 mm holes<br>Pico 2 W and MPU-6050: 2.1–2.5 mm holes | The nylon M2/M2.5/M3 kit covers all of them: M2 for the Pico and MPU-6050, M2.5 for the Pi and LiDAR, M2.5 or M3 for the MDD3A. |
| Software support | Pico 2 W, MPU-6050, MDD3A, RPLIDAR C1 | Pico 2 W, MPU-6050 and C1 are in linorobot2's Jazzy source. The MDD3A's two-PWM inputs match linorobot2's two-PWM driver mode (`USE_BTS7960_MOTOR_DRIVER`), but linorobot2 doesn't name the board. |

## Wiring overview

```
3S LiPo ── 10 A fuse ──┬── 5 V 5 A converter ── USB-C ── Pi 5 ──USB── Pico 2 W
                       │                                   └──USB── RPLIDAR C1
                       └── rocker switch ── MDD3A ──┬── left motor
                                                    └── right motor
```

- **Motor connector:** each motor has a 6-pin connector labelled **M1, GND, C1, C2, VCC, M2**. M1 and M2 go to the MDD3A's motor output. C1 and C2 are the encoder A/B signals. VCC and GND power the encoder.
- **Pico to MDD3A:** two PWM pins per motor.
- **Encoders:** C1/C2 go to the Pico on consecutive GPIO pairs, because linorobot2's PIO encoder needs pairs.
- **Encoder power:** a separate AMS1117-3.3 module, fed from the Pico's VBUS pin (5 V from the Pi's USB). Don't use the Pico's own 3V3 pin.
- **IMU:** MPU-6050 on the Pico's I2C (GP4/GP5), address 0x68. The Adafruit board has 10 kΩ pull-ups; add 2 kΩ pull-ups to 3.3 V as linorobot2 requires.
- **Battery sensing:** a 33k/10k divider from the battery to GP28, which keeps the pin below 3.3 V.
- **Grounds and capacitors:** one common ground for everything, and 220–470 µF capacitors across the motor and logic supply lines.
- **Rocker switch:** cuts motor power only. The Pi and Pico stay up, so you can stop the motors without losing the robot's state. Mount it on the top deck, where you can reach it without reaching past the wheels.
- **Bumper:** two microswitches on Pico GPIO pins, each wired with its normally-open contact to ground and the Pico's internal pull-up enabled. The firmware stops the motors whenever one is pressed, whatever the Pi is commanding.
- **Low-battery cutoff:** the Pico reads the battery through the divider on GP28.
  - At about 3.5 V per cell, it stops the motors.
  - At about 3.4 V per cell, it tells the Pi to shut down. This matters because the converter keeps the Pi running down to 8 V (about 2.7 V per cell), which damages a LiPo.
  - The beeping alarm stays on as a backup.
- **MDD3A's 5 V output:** it gives only 200 mA, so don't power the Pi or the LiDAR from it.

## Bench tests

These haven't been run yet. Run them before soldering, cutting or crimping anything, so the parts stay returnable. Most Amazon return windows close around 17–19 November 2026, and the motors' around 23 November. The one exception is the MPU-6050, whose header pins are fine to solder.

**Start with the batteries.** Look for puffing, dents or damaged wires, then plug a voltage alarm onto each pack's balance plug.
- Expect 3.7–3.9 V per cell, with all three within about 0.05 V of each other. Return a pack if any cell is under 3.0 V or far from the others.
- Don't charge them until you first need them. Do that first charge inside the LiPo bag, on a hard surface, and stay nearby.

1. **Pi power:**
   - Flash the card with [Raspberry Pi Imager](https://www.raspberrypi.com/software/): Raspberry Pi 5, then Other general-purpose OS → Ubuntu → **Ubuntu Server 24.04.x LTS (64-bit)**. Set the hostname, user, WiFi and SSH in its customisation step. Don't use 26.04, which ROS 2 Jazzy doesn't support. If Imager doesn't list 24.04, the image is on [cdimage.ubuntu.com](https://cdimage.ubuntu.com/releases/24.04/release/).
   - Boot from your USB-C power supply, run `sudo apt update && sudo apt full-upgrade -y`, add `usb_max_current_enable=1` to `/boot/firmware/config.txt`, and reboot.
   - Set the bench supply to 12 V, clip it to the converter's input wires (no soldering), and plug the converter into the Pi.
   - Run `stress-ng --cpu 4 --timeout 120s` (install it with `sudo apt install stress-ng`), and meanwhile check `sudo vcgencmd get_throttled` and `sudo vcgencmd pmic_read_adc EXT5V_V`. If `vcgencmd` is missing, install `libraspberrypi-bin`, or `raspi-utils` if that package doesn't exist.
   - Pass: `throttled=0x0` throughout, and the 5 V reading stays at 4.8 V or above.
2. **LiDAR on the laptop:** plug it in through your USB-C adapter. WSL2 doesn't see USB devices by default, so pass the adapter through with [usbipd-win](https://github.com/dorssel/usbipd-win). Run `sllidar_ros2`'s C1 launch file and check the scan in RViz. Alternatively, plug it into the Pi and view the scan from the laptop once the Pi has ROS 2.
3. **Encoders at 3.3 V:** with the Pico on a breadboard, power one encoder from the Pico's 3V3(OUT) pin, for this test only, and connect C1 and C2 to GP2 and GP3. Leave M1 and M2 unconnected. Count edges in MicroPython while turning the wheel by hand: the count should go up one way and down the other.
   - If it stays at 0, the encoder may need 5 V. Power it from VBUS with C1 disconnected from the Pico, and measure C1 with the multimeter while turning. If it swings to about 5 V, use the 10k/20k divider.
4. **Spin and stall current:** clip the bench supply to M1 and M2.
   - At 12 V with a 0.5 A limit, the motor should spin smoothly and draw about 0.12 A or less.
   - At 3 V with about a 1.5 A limit, hold the wheel still for under 2 seconds and read the current. Multiply by 4 to get the 12 V stall current. If it's over 5 A, limit the duty cycle in firmware or move to the Cytron MDD10A.
5. **Odometry constants,** for both motors:
   - Measure the wheel diameter with calipers. The listing doesn't state it; about 65 mm is expected.
   - Count encoder ticks over 10 marked wheel turns. About 2464 per turn is expected: 11 pulses × 4 edges × 56:1 gearbox.
6. **Bumper and low-battery cutoff:** once the firmware runs, lift the wheels off the bench.
   - Press each bumper switch while the wheels spin; the motors must stop.
   - Run the battery input from the bench supply and lower it past 10.5 V (3.5 V per cell); the motors must stop.
7. **MPU-6050:** solder its header pins, then wire VIN to the Pico's 3V3(OUT), GND to GND, SDA to GP4 and SCL to GP5. An I2C scan should find address 0x68. Lying flat, the Z acceleration should read about 16384 (1 g).

## Not verified

- **Motor specs:** the listing doesn't state the stall current, encoder supply voltage, pulses per revolution or wheel size. Bench tests 3–5 measure them.
- **The converter's output under load:** it's 5.0 V with no USB-PD. Reviews say it runs a Pi 5, but bench test 1 is the real check.
- **XT60 connectors:** the listing says Amass, but the brand field doesn't, so they may not be genuine.
- **RobotShop's MDD3A price:** search snippet only. Their site blocked direct reads.
- **MDD3A with linorobot2's firmware:** the inputs match, but linorobot2 doesn't name the board.
- **Estimates:** the deck and the small parts.

## Avoid

| Part | Reason |
| --- | --- |
| L298N driver | Old bipolar design that drops 2–5 V as heat, so the motors see 8–10 V |
| DRV8833 driver | 10.8 V maximum, below a fully charged 3S pack |
| TT-motor chassis kits | Slotted-disc encoders are too coarse for odometry |
| Xiaomi LDS-02 LiDAR | No ROS 2 driver; you would have to write the motor control and driver yourself |
| LSM6DSOX or ICM-20948 IMU | linorobot2's firmware doesn't support them, so you'd lose the fallback |
| microSD cards without an A1 or A2 rating | Slow random access makes the Pi sluggish. PiShop's 32 and 64 GB SanDisk Ultra cards have no rating. |
| M3-only standoff kits | Too big for the Pi 5 and the LiDAR, which take M2.5 |
| Pi 5 2 GB | Saves $45, but compiling C++ ROS packages on the Pi runs out of memory |
| Pi 5 8 GB | $245, nearly half the budget |

## Sources

- [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware), jazzy branch, and its [wiki](https://github.com/linorobot/linorobot2_hardware/wiki): supported boards, IMUs, motor drivers, wiring rules
- [linorobot2](https://github.com/linorobot/linorobot2), jazzy branch: supported LiDARs
- [RPLIDAR C1 datasheet](https://files.waveshare.com/wiki/RPLIDAR-C1/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf): power, startup current, mounting
- [Cytron MDD3A datasheet](https://cdn.shopify.com/s/files/1/0176/3274/files/MDD3A_Datasheet.pdf): inputs, logic levels, mounting
- [Raspberry Pi 5 mechanical drawing](https://pip-assets.raspberrypi.com/categories/892-raspberry-pi-5/documents/RP-008347-DS-1-raspberry-pi-5-mechanical-drawing.pdf) and [Pico 2 W datasheet](https://pip-assets.raspberrypi.com/categories/1088-raspberry-pi-pico-2-w/documents/RP-008304-DS-2-pico-2-w-datasheet.pdf): mounting holes
- [Raspberry Pi power supply documentation](https://github.com/raspberrypi/documentation/blob/master/documentation/asciidoc/computers/raspberry-pi/power-supplies.adoc): the 600 mA USB limit and `usb_max_current_enable`
- [Ubuntu 24.04 release notes](https://documentation.ubuntu.com/release-notes/24.04/): Raspberry Pi 5 support
- Raspberry Pi 2026 price increases: [CNX Software, April 2026](https://www.cnx-software.com/2026/04/01/raspberry-pi-4-3gb-launched-for-83-75-further-price-increases-announced-across-the-board), [OMG! Ubuntu, October 2026](https://www.omgubuntu.co.uk/2026/10/raspberry-pi-2gb-costs-how-much-now)
- Prices and stock: retailer product pages read on 17 October 2026, except where noted as search snippets
