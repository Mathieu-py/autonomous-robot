# Milestone 3: See

Finished on 27 November 2026. The robot maps a room with its LiDAR, SLAM Toolbox and our EKF. On its first map, five distances from 0.15 to 2.9 m agreed with a tape measure to within 4 cm. Four of the five differences are well inside their uncertainty of ±22 mm, which comes from the map's 5 cm cells and the approximate tape readings; the fifth, the box's length, is short because the map doesn't show the box's far end.

**Done when** ([build plan](build-plan.md)): a saved map of a room.

| Check | Measured | Data |
| --- | --- | --- |
| A saved map of a room | 5.65 × 9.75 m at 5 cm per cell: a T-shaped room, and a hallway seen through its doorway | [Map run 01](../results/maps/2026-11-27-run01-first-map.png) |
| The map against a tape measure | Five distances, 0.15–2.9 m: −36 to +8 mm, 12 mm on average, each ±22 mm (one standard deviation) | [Maps](../results/README.md#maps-maps) |
| The LiDAR | 10.0 Hz, health OK; no undervoltage on the battery with it spinning | [Pi setup, section 9](pi-setup.md#9-the-lidar) |
| The laser's position and direction | A box in front of the robot shows in front of it in RViz; after a 180° turn on the spot, the walls landed within 1 mm (median) of where they were, with no extra turn needed to line them up | Pi setup, section 9 |

## What was built

**A mount for the LiDAR** ([cad/layout.py](../cad/layout.py)): a 76 × 89 × 4 mm PLA plate with R6 corners, on four 35 mm M3 pillars (20 + 15 mm standoffs), 11.5 mm above the Pi, which puts the laser 151.8 mm above the floor. The plate reaches 6 mm past each pillar. The LiDAR's M2.5 × 6 screws sit in counterbores, so they reach 3.5 mm into it, under Slamtec's 4 mm limit. Its USB adapter board lies along the LiDAR's left side on 6 mm M2 standoffs, below the laser. The Pi was turned so its USB ports face left, toward the adapter, which keeps the cables inside the deck and below the laser.

**The LiDAR on the Pi** ([Pi setup, section 9](pi-setup.md#9-the-lidar)): Slamtec's `sllidar_ros2` driver, built from source because it isn't in Jazzy's packages, on a fixed `/dev/lidar` name. `robot.launch.py` starts it with everything else; `lidar:=false` leaves it out when replaying a recording.

**The laser's frame** ([robot.urdf.xacro](../ros/robot_description/urdf/robot.urdf.xacro)): its position from the CAD, 45 mm behind the axle, 10 mm right of centre and 151.8 mm up, turned 180°. The turn came from checking where a box in front of the robot, then beside it, showed up.

**SLAM Toolbox** ([slam.launch.py](../ros/robot_description/launch/slam.launch.py), [config/slam.yaml](../ros/robot_description/config/slam.yaml)): its default online, asynchronous mapping, with two changes: our `base_link` frame and the C1's 12 m range. It runs on the Pi; the map is watched and saved from the laptop.

**A gyro recheck first** ([square run 05](../results/README.md#square-tests-squares)): before the LiDAR went on, two turns each way on another day read as on 26 November, which closed milestone 2's open question about the gyro.

## What went wrong and what it taught

- **The LiDAR's x axis points at its cable.** Slamtec's driver puts the scan's 0° toward the cable side, not toward the arrow on top. A box in front of the robot showed up on the laser's −x axis, and one to its left on −y: a plain 180° turn, not a mirror image.
- **SLAM Toolbox's launch file assumes a simulator.** It defaults to simulation time, so on the robot it would wait forever for a simulated clock; our launch file sets real time. Its default config also expects a `base_footprint` frame, which this robot doesn't have.
- **The first plate design.** Its pillar holes were 3 mm from the edges, leaving 1.3 mm of plastic beside each; the LiDAR's screws through the 4 mm plate would have gripped only 2 mm; and it hung out behind the Pi to put the adapter by rear-facing USB ports. The review widened it and added counterbores, and once the Pi was turned, the adapter moved to the LiDAR's side.
- **udev finishes after its command returns.** `ls /dev/lidar` straight after `udevadm trigger` found nothing; `udevadm settle` waits for it.
- **Measuring on a map needs straight walls.** One tape line ended at a corner, and a line fitted to the cells there mixed two walls and gave 1.319 m. Measured on the wall's straight part just below the corner, the map gives 1.225 m, against 1.235 m by tape.
- **Scans arrive ahead of the pose.** RViz occasionally drops a scan with "extrapolation into the future": our EKF's pose ran about 40 ms behind the sensors, because it sorted their readings by time with a 30 ms margin; since 28 November it runs a few ms behind. SLAM Toolbox waits up to 0.2 s for the pose, so it isn't affected.

## Not done yet

- **Finer measurements need a finer map.** At 5 cm per cell, a wall is placed to within about 2.5 cm; a 2.5 cm map would halve that, with four times the cells. Nav2 works at 5 cm.
- **Loop closure hasn't been tested on its own.** This was one loop of one room. A bigger area, or several rooms, would show whether SLAM Toolbox corrects drift when the robot comes back to a place it has mapped.
- **Only the map image is saved,** not SLAM Toolbox's pose graph, which it would need to carry on mapping later. Nav2 only needs the image.
- **The LiDAR's weight, and carpet** (from milestone 2): the square test hasn't been repeated with the LiDAR on.
