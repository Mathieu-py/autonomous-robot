# Autonomous Mobile Robot — Build Plan

Build a small differential-drive robot and write its software layer by layer, until it maps a room by itself and drives to any point clicked on the map. The parts were ordered on 17 October 2026 and have arrived; the planner and path follower get written in simulation while the robot is built. Parts and your own basic tools cost about CAD $700 before tax; [parts-list.md](parts-list.md) has the exact parts and links.

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

Six milestones, each a working capability, and each ends with one measurement written up in the repo. Milestones 4 to 6 can start in simulation while the robot is built; 1 to 3 need the robot.

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

While the robot is built, the planner, path follower and avoidance get written and tested in Gazebo, so the finished robot starts with code that already works. Once milestones 1–3 run on the robot, that code moves over with retuning.

## Hardware and budget

Build a two-wheel-drive robot on the [linorobot2](https://github.com/linorobot/linorobot2) reference design, but write the motor firmware from scratch. linorobot2 supplies a proven parts list, wiring and robot description, and has Jazzy support. Its [firmware](https://github.com/linorobot/linorobot2_hardware) already runs on the Pico 2 W and ESP32 with an MPU-6050 or BNO085 IMU, so it is the fallback if the custom firmware stalls.

TurtleBot 4 and 3 are ruled out: the Create 3 base and the Dynamixel servos hide the motor control this project should own.

| Part | Selection | Approx. CAD | Notes |
| --- | --- | --- | --- |
| Compute | Raspberry Pi 5 (4 GB), active cooler, 32 GB microSD, micro-HDMI cable | 198 | Runs Ubuntu 24.04 and Jazzy; the 8 GB costs $90 more after 2026's price rises |
| Chassis and motors | 2× JGB37-520 12 V encoder gear motors with wheels, ball caster, 3D-printed deck, standoffs, bumper microswitches | 94 | 2464 counts per wheel turn; poor encoders and wheel slip spoil every localization measurement |
| LiDAR | RPLIDAR C1 | 94 | Time-of-flight, 12 m; supported by linorobot2 |
| Battery and power | 2× 3S 2200 mAh LiPo, 5 V 5 A converter, fuse, physical motor cutoff | 111 | |
| Microcontroller, driver, IMU | Pico 2 W and its USB cable; Cytron MDD3A (3 A continuous, 5 A peak); MPU-6050 | 50 | All supported by linorobot2's firmware |
| Charging and safety | Balance charger, LiPo bag, low-voltage alarm | 52 | |
| Wiring and small parts | Perfboard, wire, 3.3 V regulators, resistors, capacitors | 47 | Estimate |
| Tools you keep | Soldering iron kit, multimeter | 61 | Bench supply, oscilloscope and calipers borrowed from the DEN |
| **Total** | | **about 700** | Before tax and shipping |

Prices were checked on 17 October 2026 and exclude tax and shipping. The overhead ground-truth camera is dropped from the budget: a phone looking down on the room, with printed AprilTags, does the same job.

[parts-list.md](parts-list.md) turns this table into exact part numbers, with links, and checks that they fit together: motor voltage against the battery, the driver's current rating against the motors' stall current, encoder signal levels against the Pico's 3.3 V logic, and a 5 V, 5 A supply for the Pi 5. Three things still need a bench test before anything gets soldered: the motors' real stall current, whether the encoders run at 3.3 V, and the converter's output under load.

## Computing setup

Use the existing Dell XPS 14 (32 GB RAM, RTX 4050) under WSL2 with Ubuntu 24.04 and ROS 2 Jazzy. This is set up and working; switch to native Ubuntu only if WSL2 causes problems later.

- **Day-1 check:** the risk is Gazebo's simulated GPU LiDAR, not the GUI. Under WSLg it can quietly fall back to software rendering and slow the simulation. `glxinfo -B` must name D3D12/NVIDIA, not llvmpipe, and Gazebo's real-time factor should stay near 1.0 with the LiDAR running. **Passed on 4 November 2026:** the Nav2 TurtleBot demo reached its goals at a real-time factor of about 1.0, with its GPU LiDAR running on the RTX 4050.
- **GPU settings:** this laptop has both Intel and NVIDIA GPUs. Without the first line below, `glxinfo -B` reports llvmpipe; without the second, it picks the Intel GPU. Both go in `~/.bashrc`:
  ```bash
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
  ```
- **Nav2 demo:** set the initial pose within about a minute of launching `tb3_simulation_launch.py`. Otherwise Nav2 stops starting up and Navigation stays `inactive`, and the demo needs a restart. The robot starts at (-2, -0.5), facing +x.
- **Fallback:** if WSL2 still fights after a day or two, boot Ubuntu 24.04 from an external USB-C SSD. That gives the native GPU and network, and the Windows install is untouched.
- **Laptop-to-robot networking:** WSL2's default NAT mode hides Linux behind its own virtual adapter, which blocks ROS 2 discovery from the robot. Set `networkingMode=mirrored` under `[wsl2]` in `.wslconfig`; Microsoft lists multicast support among its benefits on Windows 11 22H2 and later ([WSL networking docs](https://learn.microsoft.com/en-us/windows/wsl/networking)). A Fast DDS discovery server or rmw_zenoh is the fallback. This isn't set yet, because simulation doesn't need it: do it when connecting the laptop to the Pi. Applying it means running `wsl --shutdown`, which also restarts Docker Desktop.
- **Workspace:** keep it in the Linux filesystem (`~/robot_ws`), not under `/mnt/c`. It's created; clone this repo into `~/robot_ws/src` and work from that copy. The Pi gets its own workspace and clone. The Pico firmware lives in the same repo, in a folder colcon skips, because it's built with the Pico SDK.
- **ROS version:** stay on Jazzy even though Lyrical Luth shipped in May 2026 ([ROS 2 release notes](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html)). Nav2, drivers and tutorials are more mature on Jazzy, and Raspberry Pi 5 with Ubuntu 24.04 is well trodden.

## Safety

Treat the first powered runs as tests of the safety features themselves.

- A physical switch that cuts motor power, on the top deck and within reach
- A firmware watchdog that stops the motors when commands stop
- A front bumper of microswitches wired to the Pico, which stops the motors without the Pi; a 2D LiDAR misses obstacles below or above its scan plane
- A fused battery, with a firmware low-battery cutoff that stops the motors and shuts the Pi down, and a beeping alarm as backup
- Low speed and supervised runs until the safety stop works

## Risks

The likeliest problem is hardware bring-up taking longer than planned; the simulation work keeps going meanwhile.

| Risk | Mitigation |
| --- | --- |
| Hardware bring-up takes longer than planned | Simulation work continues in parallel; linorobot2's firmware is the fallback |
| Odometry too poor for good maps | Encoder motors; calibrate wheel radius and track width with square-path runs |
| Simulation code behaves differently on the robot | Expect retuning; run the same test in simulation and on the robot |
| WSL2 falls back to software rendering | Both GPU settings in `~/.bashrc`; recheck with `glxinfo -B`; Ubuntu on an external SSD |
| Robot and laptop can't discover each other | Mirrored networking; discovery server or rmw_zenoh; SSH and onboard logging first |
| Time competes with coursework and recruiting | Each milestone is a working result on its own; push to a public repo at each one |

## First actions

The first two weeks get the parts ordered and the simulation running.

- [x] Pick exact part numbers and check they fit together (see [parts-list.md](parts-list.md))
- [x] Order the parts, motors first (ordered 17 October 2026)
- [x] Install WSL2 with Ubuntu 24.04, ROS 2 Jazzy and Gazebo Harmonic
- [x] Run `glxinfo -B` and a Gazebo world with a GPU LiDAR; check the renderer and real-time factor
- [x] Run the Nav2 TurtleBot simulation and send it a goal, to see a full stack working
- [x] Create the repo and a ROS 2 workspace
- [ ] Clone this repo into `~/robot_ws/src` and work from that copy
- [ ] Run the [bench tests](parts-list.md#bench-tests) before soldering anything; most Amazon return windows close around 17–19 November 2026
- [ ] Write A* in C++ on a saved map, with unit tests
- [ ] Write the pure-pursuit follower and drive the simulated robot along your own paths

## Sources

- Autonomous Mobile Robot Project Brief, 15 October 2026 (the original brief)
- [parts-list.md](parts-list.md): part prices and links, checked 17 October 2026
- [linorobot2](https://github.com/linorobot/linorobot2) and [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware): supported distributions, boards, IMUs and LiDARs
- [Microsoft WSL networking docs](https://learn.microsoft.com/en-us/windows/wsl/networking): mirrored mode and multicast
- [Nav2 Getting Started](https://docs.nav2.org/getting_started/): install and the TurtleBot demo
- [Gazebo on WSLg](https://discourse.openrobotics.org/t/wslg-with-gpu-support-available-on-latest-version-of-gazebo-garden-and-harmonic/48128): GPU support, not an officially tested platform
- [ROS 2 Lyrical Luth release notes](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html)
