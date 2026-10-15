# Autonomous Mobile Robot — Build Plan

Build a small differential-drive robot and write its software layer by layer, until it maps a room by itself and drives to any point clicked on the map. Order the parts now, and write the planner and path follower in simulation while they ship. Parts cost roughly CAD $700–1,000.

## Final demo

One unedited take, with no intervention beyond the final click:

1. Place the robot in a room it has never seen.
2. It explores on its own and builds the map.
3. It picks unexplored regions and chooses where to go next.
4. It avoids an obstacle placed in its path mid-run.
5. It finishes exploring and returns to its start.
6. A destination is clicked on the map, and it drives there.

An unedited take convinces more than a polished edit. Record a short clip at each milestone along the way.

## Milestones

Six milestones, each a working capability, and each ends with one measurement written up in the repo. Milestones 4 to 6 can start in simulation while the parts ship; 1 to 3 need the robot.

| # | Milestone | You write | Reuse | Done when |
| --- | --- | --- | --- | --- |
| 1 | Drive it | Motor firmware: encoder reads, wheel-speed PID, watchdog; a teleop node | Pico 2 W SDK; linorobot2's firmware as a reference | Each wheel holds a commanded speed, with the PID step response plotted; the motors stop when commands stop |
| 2 | Know where it is | Wheel odometry; your own EKF fusing the IMU | robot_localization, as the yardstick | Drift measured after driving a square; your EKF compared with robot_localization |
| 3 | See | Robot description (URDF) and transforms | LiDAR driver, SLAM Toolbox | A saved map of a room |
| 4 | Go places | A* with footprint inflation; a pure-pursuit path follower | Nav2, for comparison | Click a goal in RViz and it drives there; path length and time compared with Nav2 |
| 5 | Don't hit things | Local obstacle avoidance (e.g. DWA-style velocity sampling) and a safety stop | Nav2's collision monitor, for comparison | Drives around a box dropped in its path; stops for a hand in front of the LiDAR |
| 6 | Explore | Frontier-based exploration | Your own stack from milestones 1–5 | The final demo runs start to finish |

**Stretch goals,** after milestone 6: your own occupancy-grid mapper, built from SLAM Toolbox's poses (far smaller than writing full SLAM); and the Wi-Fi-fault speed governor from the earlier research version, which is in the online doc's version history.

## Order of work

Rough pacing at 10–15 hours a week; numbers match the milestones table.

| | Weeks 1–2 | Weeks 3–4 | Weeks 5–6 | Weeks 7 onward |
| --- | --- | --- | --- | --- |
| **Simulation** (laptop) | Setup: WSL2, Jazzy, Gazebo, Nav2 demo | 4 · Go places: A* planner, pure pursuit | 5 · Avoid: local avoidance, safety stop | 6 · Explore: frontiers, whole room in sim |
| **Robot** (hardware) | Parts ship: order on day 1, test each part | 1 · Drive it: firmware, PID, teleop | 2–3 · Localize: odometry, EKF, LiDAR, map | **Port 4–6:** sim code on the robot, explore a room |

While the parts ship, the planner, path follower and avoidance get written and tested in Gazebo, so the robot arrives to code that already works. Once milestones 1–3 run on the robot, that code moves over with retuning.

## Hardware and budget

Build a two-wheel-drive robot on the [linorobot2](https://github.com/linorobot/linorobot2) reference design, but write the motor firmware from scratch. linorobot2 supplies a proven parts list, wiring and robot description, and has Jazzy support. Its [firmware](https://github.com/linorobot/linorobot2_hardware) already runs on the Pico 2 W and ESP32 with a BNO085 IMU, so it is the fallback if the custom firmware stalls.

TurtleBot 4 and 3 are ruled out: the Create 3 base and the Dynamixel servos hide the motor control this project should own.

| Part | Selection | Approx. CAD | Notes |
| --- | --- | --- | --- |
| Compute | Raspberry Pi 5 (8 GB), cooling, storage | 275 | Runs Ubuntu 24.04 and Jazzy |
| Chassis and motors | Rigid 2WD chassis, geared DC motors with quadrature encoders, caster | 170 | Spend here: poor encoders and wheel slip spoil every localization measurement |
| LiDAR | RPLIDAR C1 or LDROBOT LD19 | 150 at most | Both supported by linorobot2; expected below the A1's price |
| Battery and power | Protected battery, regulators, fuse, physical motor cutoff | 100 | |
| Microcontroller, driver, IMU | Pico 2 W or ESP32; dual-channel driver rated for stall current; BNO085 | 85 | All supported by linorobot2's firmware |
| Ground truth | Overhead camera and printed AprilTags | 50 | Optional; measures drift precisely |
| Mounting, wiring, contingency | | 100 | |
| **Total** | | **about 930** | Within the original CAD $700–1,000 range |

Prices are the original brief's estimates plus the camera line; none was re-quoted.

Before ordering, turn the table into exact part numbers and check that they fit together: motor voltage against the battery, the driver's current rating against the motors' stall current, and encoder signal levels against the microcontroller's 3.3 V logic. The Pi 5 also needs a regulator that supplies 5 V at up to 5 A. Starting from linorobot2's tested parts cuts this work down.

## Computing setup

Start on the existing Dell XPS 14 (32 GB RAM, RTX 4050) under WSL2 with Ubuntu 24.04 and ROS 2 Jazzy. Switch to native Ubuntu only if the day-1 rendering check fails.

- **Day-1 check:** the risk is Gazebo's simulated GPU LiDAR, not the GUI. Under WSLg it can quietly fall back to software rendering and slow the simulation. `glxinfo -B` must name D3D12/NVIDIA, not llvmpipe, and Gazebo's real-time factor should stay near 1.0 with the LiDAR running.
- **Fallback:** if WSL2 still fights after a day or two, boot Ubuntu 24.04 from an external USB-C SSD. That gives the native GPU and network, and the Windows install is untouched.
- **Laptop-to-robot networking:** WSL2's default NAT mode hides Linux behind its own virtual adapter, which blocks ROS 2 discovery from the robot. Set `networkingMode=mirrored` under `[wsl2]` in `.wslconfig`; Microsoft lists multicast support among its benefits on Windows 11 22H2 and later ([WSL networking docs](https://learn.microsoft.com/en-us/windows/wsl/networking)). A Fast DDS discovery server or rmw_zenoh is the fallback.
- **Workspace:** keep it in the Linux filesystem (`~/robot_ws`), not under `/mnt/c`.
- **ROS version:** stay on Jazzy even though Lyrical Luth shipped in May 2026 ([ROS 2 release notes](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html)). Nav2, drivers and tutorials are more mature on Jazzy, and Raspberry Pi 5 with Ubuntu 24.04 is well trodden.

## Safety

Treat the first powered runs as tests of the safety features themselves.

- A physical switch that cuts motor power, within reach
- A firmware watchdog that stops the motors when commands stop
- A protected battery with a fuse
- Low speed and supervised runs until the safety stop works

## Risks

The likeliest problem is hardware bring-up taking longer than planned; the simulation work keeps going meanwhile.

| Risk | Mitigation |
| --- | --- |
| Hardware bring-up takes longer than planned | Simulation work continues in parallel; linorobot2's firmware is the fallback |
| Odometry too poor for good maps | Encoder motors; calibrate wheel radius and track width with square-path runs |
| Simulation code behaves differently on the robot | Expect retuning; run the same test in simulation and on the robot |
| WSL2 falls back to software rendering | Day-1 check; Ubuntu on an external SSD |
| Robot and laptop can't discover each other | Mirrored networking; discovery server or rmw_zenoh; SSH and onboard logging first |
| Time competes with coursework and recruiting | Each milestone is a working result on its own; push to a public repo at each one |

## First actions

The first two weeks get the parts ordered and the simulation running.

- [ ] Pick exact part numbers, check they fit together (see Hardware and budget), then order
- [ ] Install WSL2 with Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic
- [ ] Run `glxinfo -B` and a Gazebo world with a GPU LiDAR; check the renderer and real-time factor
- [ ] Run the Nav2 TurtleBot simulation and send it a goal, to see a full stack working
- [ ] Create the repo and a ROS 2 workspace
- [ ] Write A* in C++ on a saved map, with unit tests
- [ ] Write the pure-pursuit follower and drive the simulated robot along your own paths

## Sources

- Autonomous Mobile Robot Project Brief, 15 October 2026 (the original brief)
- [linorobot2](https://github.com/linorobot/linorobot2) and [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware): supported distributions, boards, IMUs and LiDARs
- [Microsoft WSL networking docs](https://learn.microsoft.com/en-us/windows/wsl/networking): mirrored mode and multicast
- [ROS 2 Lyrical Luth release notes](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html)
