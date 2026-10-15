# Parts List — Budget Build

The parts cost about **CAD $591 before tax and shipping**. That is down from the build plan's original estimate of about $930, and about $90 over the $500 target. Raspberry Pi prices rose roughly 40% during 2026, so the Pi 5 4GB alone is now $155. This is the cheapest set that still meets two conditions:

- linorobot2's firmware supports every part, so it remains the fallback.
- The encoders and IMU are good enough for the odometry and EKF in milestone 2.

Prices were checked on 15 October 2026. Borrow the soldering iron, multimeter, bench power supply and calipers from a makerspace. Your SYDE 192L kit already covers the breadboard, jumper wires, wire cutters, buttons and LEDs.

## Shopping list

### CanaKit

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Raspberry Pi 5, 4 GB | 154.95 | [canakit.com](https://www.canakit.com/raspberry-pi-5-4gb.html) | Onboard computer: Ubuntu 24.04, ROS 2 Jazzy, SLAM, navigation. PiShop was out of stock. |
| Raspberry Pi 5 Active Cooler | 6.95 | [canakit.com](https://www.canakit.com/raspberry-pi-5-active-cooler.html) | Prevents the Pi from slowing down when it overheats under SLAM load |

### PiShop.ca

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Raspberry Pi Pico 2 W | 9.80 | [pishop.ca](https://www.pishop.ca/product/raspberry-pi-pico-2-w/) | Motor microcontroller: encoders, wheel PID, watchdog. linorobot2 env `pico2w`. Sold without headers. |
| Adafruit MPU-6050 (#3886) | 18.85 | pishop.ca, search "3886" | IMU for the EKF. Genuine chip. linorobot2 `USE_MPU6050_IMU`. |

### DigiKey.ca

| Part | Price | Link | Used for |
| --- | --- | --- | --- |
| Cytron MDD3A dual motor driver | 15.81 | [digikey.ca](https://www.digikey.ca/en/products/result?keywords=MDD3A) | 4–16 V; 3 A continuous / 5 A peak per channel; 3.3 V logic; reverse-polarity protection |

DigiKey charges shipping on small orders. Before ordering, check whether PiShop or RobotShop has the MDD3A (RobotShop is about $12.50, from a search snippet). Otherwise, add the small parts below to this order.

### Amazon.ca

| Part | Qty | Price | Link | Used for |
| --- | --- | --- | --- | --- |
| RPLIDAR C1 (Waveshare listing) | 1 | 93.99 | [B0CT31PH8S](https://www.amazon.ca/dp/B0CT31PH8S) | 2D LiDAR: time-of-flight, 12 m range, 10 Hz. USB adapter included. linorobot2 `--laser c1`. |
| JGB37-520 12 V 178 RPM motor with Hall encoder, wheel, bracket and coupling | 2 | 38.66 (19.33 each) | [B0H4RH7MRN](https://www.amazon.ca/dp/B0H4RH7MRN) | Drive motors. 56:1 gearbox, 2464 counts per wheel turn. **Ships from China, 1–2 weeks.** |
| Zeee 3S 2200 mAh 50C LiPo, XT60, 2-pack | 1 | 42.29 | [B0D5LKXHBK](https://www.amazon.ca/dp/B0D5LKXHBK) | Main battery. The second pack charges while the first runs. |
| PARKTION 12/24 V to 5 V 5 A USB-C converter | 1 | 13.99 | [B0H7S63FXC](https://www.amazon.ca/dp/B0H7S63FXC) | Powers the Pi 5. 8–32 V input, so it keeps working down to a flat battery. |
| Inline blade-fuse holders, 14 AWG, 3-pack | 1 | 9.99 | [B0D47LJHTW](https://www.amazon.ca/dp/B0D47LJHTW) | 10 A main fuse at the battery |
| DaierTek mini rocker switch | 1 | 11.99 | [B07S2QJKTX](https://www.amazon.ca/dp/B07S2QJKTX) | Physical motor-power cutoff |
| Amass XT60 connectors, 10 pairs | 1 | 10.99 | [B07233RVL6](https://www.amazon.ca/dp/B07233RVL6) | Battery connector on the robot side |
| Metal ball casters, 5 pcs | 1 | 11.59 | [B0HHDJ47JL](https://www.amazon.ca/dp/B0HHDJ47JL) | Third wheel |
| M3 brass standoff kit | 1 | 13.99 | [B018C19KJ0](https://www.amazon.ca/dp/B018C19KJ0) | Mounting the boards; levelling the caster with the wheel axles |
| ShunLong 2–3S balance charger | 1 | 23.99 | [B0HDMXHFRH](https://www.amazon.ca/dp/B0HDMXHFRH) | LiPo charging |
| LiPo safe bag | 1 | 15.50 | [B0F6LP63HD](https://www.amazon.ca/dp/B0F6LP63HD) | Charging and storing the batteries |
| LiPo low-voltage alarm, 2-pack | 1 | 12.96 | [B073WXVRBT](https://www.amazon.ca/dp/B073WXVRBT) | Beeps before a cell drops low enough to be damaged. Use it until your firmware monitors the battery. |

Before ordering the motors, confirm that the listing is set to the **12 V, 178 RPM** variant with the Hall encoder. If the gear ratio isn't stated, ask the seller for it. Motors sold as "107 RPM" and "110 RPM" are different motors, and the ratio determines the counts per turn.

### Anywhere (estimates)

Many of these may be free at the makerspace.

| Part | Approx. price | Used for |
| --- | --- | --- |
| microSD card, 32 GB, A1 or A2 | 20 | Pi operating system. Cards are expensive this year: the official 64 GB card is $44.95. |
| Deck: 3D-printed plate, or 1/4" plywood | 10 | Chassis. Print the motor, LiDAR and caster mounts. |
| Wiring and small parts:<br>• perfboard<br>• 16–18 AWG silicone wire<br>• resistor assortment<br>• electrolytic capacitor assortment (220–470 µF)<br>• AMS1117-3.3 regulator modules<br>• male header pins<br>• 10 A blade fuses<br>• heat shrink | 55 | The final wiring. A breadboard shakes loose on a moving robot, so use perfboard. |

## Budget

| Group | CAD |
| --- | --- |
| Compute: Pi 5 4 GB, cooler, microSD | 181.90 |
| LiDAR | 93.99 |
| Motors and wheels | 38.66 |
| Microcontroller, driver, IMU | 44.46 |
| Battery and power: batteries, converter, fuse holders, switch, connectors | 89.25 |
| Chassis: casters, standoffs, deck | 35.58 |
| Wiring and small parts | 55.00 |
| Charging and safety: charger, LiPo bag, alarm | 52.45 |
| **Total before tax and shipping** | **591.29** |

About $85 of that is estimates: the microSD card, the deck and the small parts. Sales tax of 5–15% (depending on province) comes on top.

Optional upgrades:
- **Adafruit BNO085 IMU** instead of the MPU-6050: +$25. It does sensor fusion on the chip. Wire it to the Pico, not the Pi, because the Pi's I2C has timing trouble with the BNO08x.
- **Pi 5 8 GB:** +$90.

## Order of purchase

Order everything on day 1. The motors take longest because they ship from China, so they set the date hardware work can start (build plan weeks 3–4).

1. Amazon.ca: the motors first, then the rest of the Amazon list. The LiDAR arrives in a few days and can be tested on the laptop right away.
2. CanaKit: Pi 5 and cooler.
3. PiShop: Pico 2 W and MPU-6050. Add the MDD3A if PiShop has it.
4. DigiKey: MDD3A, if no cheaper source has it. Add the small parts to this order to make the shipping worth it.

## Compatibility checks

| Check | Parts | Result |
| --- | --- | --- |
| Motor voltage vs battery | 12 V motors, 3S LiPo (11.1 V nominal, 12.6 V full) | OK. A full charge is 5% over the motor rating, so cap the PWM duty at about 95% (12 / 12.6) in firmware. |
| Driver voltage | MDD3A (16 V max) vs 12.6 V | OK. The DRV8833 (10.8 V max) would not be. |
| Driver current vs motor stall | MDD3A 3 A continuous, 5 A peak vs listed 2.3–2.4 A stall | OK on paper. The stall figure comes from the seller, so measure it (see below). |
| Encoder signals vs 3.3 V logic | Encoders listed for 3.3–5 V supply; Pico pins take 3.3 V only | Power the encoders at 3.3 V so their outputs are 3.3 V. If they need 5 V, put a 10k/20k divider on each A/B line. |
| Pi 5 supply | PARKTION 5 V 5 A converter | OK. It doesn't negotiate USB-PD, so the Pi limits its USB ports to 600 mA total. That's enough for the C1 and the Pico; check the C1's draw on the bench. Don't override the limit with `usb_max_current_enable`. |
| Software support | Pico 2 W, MPU-6050, MDD3A, RPLIDAR C1 | Pico 2 W, MPU-6050 and C1 are in linorobot2's Jazzy source. The MDD3A should work in its two-PWM driver mode (`USE_BTS7960_MOTOR_DRIVER`), but linorobot2 doesn't name the board. |

## Wiring overview

```
3S LiPo ── 10 A fuse ──┬── 5 V 5 A converter ── USB-C ── Pi 5 ──USB── Pico 2 W
                       │                                   └──USB── RPLIDAR C1
                       └── rocker switch ── MDD3A ──┬── left motor
                                                    └── right motor
```

- **Pico to MDD3A:** two PWM pins per motor.
- **Encoders:** A/B lines go to the Pico on consecutive GPIO pairs, because linorobot2's PIO encoder needs pairs.
- **Encoder power:** a separate AMS1117-3.3 module, fed from the Pico's VBUS pin (5 V from the Pi's USB). Don't use the Pico's own 3V3 pin.
- **IMU:** MPU-6050 on the Pico's I2C (GP4/GP5), with 2 kΩ pull-ups to 3.3 V.
- **Battery sensing:** a 33k/10k divider from the battery to GP28, which keeps the pin below 3.3 V.
- **Grounds and capacitors:** one common ground for everything, and 220–470 µF capacitors across the motor and logic supply lines.
- **Rocker switch:** cuts motor power only. The Pi and Pico stay up, so you can stop the motors without losing the robot's state.

## Bench tests when parts arrive

1. **Pi power:** flash Ubuntu Server 24.04 and boot from your USB-C power supply. Then run it from the battery through the converter. `vcgencmd get_throttled` should print `throttled=0x0` while the Pi is under load.
2. **LiDAR on the laptop:** WSL2 doesn't see USB devices by default, so pass the adapter through with [usbipd-win](https://github.com/dorssel/usbipd-win). Run `sllidar_ros2`'s C1 launch file and check the scan in RViz.
3. **Encoders at 3.3 V:** power one encoder at 3.3 V and turn the wheel by hand. Watch the A and B signals on two Pico pins: both should switch between 0 and 3.3 V, a quarter-cycle apart.
4. **Stall current:** on a bench supply set to 3 V, hold the motor shaft for under 2 seconds and read the current. Multiply by 4 to get the 12 V stall current. If it's over 5 A, limit the duty cycle in firmware or move to the Cytron MDD10A.
5. **Odometry constants:**
   - Measure the wheel diameter with calipers. The listing doesn't state it; about 65 mm is expected.
   - Count encoder ticks over 10 marked wheel turns to get the real counts per turn.

## Not verified

- **JGB37-520 stall current:** the 2.3–2.4 A comes from seller listings. Nobody has published a measurement.
- **Encoder supply voltage:** listings say 3.3–5 V. Bench test 3 confirms it.
- **Wheel diameter** of the motor-set wheels.
- **MDD3A with linorobot2's firmware:** inferred from its inputs, not named in their docs.
- **Estimates:** the microSD card, the deck and the small parts.
- **RobotShop prices:** search snippets only. Their site blocked direct reads.

## Avoid

| Part | Reason |
| --- | --- |
| L298N driver | Old bipolar design that drops 2–5 V as heat, so the motors see 8–10 V |
| DRV8833 driver | 10.8 V maximum, below a fully charged 3S pack |
| TT-motor chassis kits | Slotted-disc encoders are too coarse for odometry |
| Xiaomi LDS-02 LiDAR | No ROS 2 driver; you would have to write the motor control and driver yourself |
| LSM6DSOX or ICM-20948 IMU | linorobot2's firmware doesn't support them, so you'd lose the fallback |
| Pi 5 2 GB | Saves $45, but compiling C++ ROS packages on the Pi runs out of memory |
| Pi 5 8 GB | $245, nearly half the budget |

## Sources

- [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware), jazzy branch, and its [wiki](https://github.com/linorobot/linorobot2_hardware/wiki): supported boards, IMUs, motor drivers, wiring rules
- [linorobot2](https://github.com/linorobot/linorobot2), jazzy branch: supported LiDARs
- Raspberry Pi 2026 price increases: [CNX Software, April 2026](https://www.cnx-software.com/2026/04/01/raspberry-pi-4-3gb-launched-for-83-75-further-price-increases-announced-across-the-board), [OMG! Ubuntu, October 2026](https://www.omgubuntu.co.uk/2026/10/raspberry-pi-2gb-costs-how-much-now)
- Prices: retailer product pages read on 15 October 2026, except where noted as search snippets
