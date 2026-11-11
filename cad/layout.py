"""Where each part sits, in millimetres: option B (chosen 2026-11-10).

Motors sit on the lower deck in printed mounts; the wheels turn through slots,
between the two decks. Origin at the deck centre on the wheel axis, z = 0 on
the floor. +x is forward, +y is left. Parts are (centre x, centre y, size x,
size y).

Heavy parts sit behind the axle: with the caster at the back, a centre of
mass too close to the axle tips the robot forward when it brakes. Estimated
centre of mass: 16 mm behind the axle and 54 mm up, safe to 2.8 m/s².
"""
from dimensions import (BATTERY_LENGTH, BATTERY_WIDTH, CASTER_FLANGE_LENGTH,
                        CASTER_FLANGE_WIDTH, CASTER_HEIGHT, CONVERTER_LENGTH,
                        CONVERTER_TAB_LENGTH, CONVERTER_WIDTH,
                        GEARBOX_FACE_TO_WHEEL, IMU_LENGTH,
                        IMU_WIDTH, MDD3A_LENGTH, MDD3A_WIDTH, MOTOR_LENGTH,
                        MICROSWITCH_HOLES_TO_ROLLER_SIDE, MICROSWITCH_WIDTH,
                        PI_LENGTH, PI_WIDTH, WHEEL_DIAMETER, WHEEL_WIDTH)

# --- Drive geometry ---
ENCODER_GAP = 6.0                    # between the two motors' back ends
GEARBOX_FACE_Y = ENCODER_GAP / 2 + MOTOR_LENGTH
WHEEL_INNER_Y = GEARBOX_FACE_Y + GEARBOX_FACE_TO_WHEEL
WHEEL_CENTRE_Y = WHEEL_INNER_Y + WHEEL_WIDTH / 2
WHEEL_OUTER_Y = WHEEL_INNER_Y + WHEEL_WIDTH
WHEEL_SEPARATION = 2 * WHEEL_CENTRE_Y
AXLE_Z = WHEEL_DIAMETER / 2
SHAFT_OFFSET = 6.9                   # gearbox centre above the shaft (measured)
GEARBOX_CENTRE_Z = AXLE_Z + SHAFT_OFFSET
GEARBOX_LENGTH = 24.0                # measured

# --- Decks ---
DECK_RADIUS = 120.0                  # leaves a ring outside the wheel slots
DECK_THICKNESS = 4.0
LOWER_DECK_BOTTOM = CASTER_HEIGHT    # the caster bolts straight on
LOWER_DECK_TOP = LOWER_DECK_BOTTOM + DECK_THICKNESS
UPPER_DECK_BOTTOM = 80.0             # 12 mm above the wheel tops
WHEEL_SLOT_HALF_LENGTH = 35.5        # along x; wheel chord through the deck plus margin
WHEEL_SLOT_MARGIN = 3.0              # each side of the tyre, along y
GEARBOX_CRADLE_DEPTH = 2.0           # pocket in the deck top under each gearbox

# --- Printed motor mounts (PETG), left side; the right mirrors it ---
MOUNT_THICKNESS = 5.0
MOUNT_ARCH_RADIUS = 23.0            # upright outline: an arch round the gearbox centre
MOUNT_FEET_X = (20.5, 32.0)          # |x| range of each foot, clear of the gearbox
MOUNT_FEET_Y = (34.0, GEARBOX_FACE_Y)
MOUNT_HOLES = [(sx * 26.25, y) for sx in (1, -1) for y in (39.0, 47.0)]  # M3 x 15, full 5 mm foot under the head
GEARBOX_HOLE_CIRCLE = 31.0           # 6 x M3, 4 mm deep (listings)
# Gearbox screws are M3 x 6, their heads in counterbores so they reach 3.95 mm
# into the 4 mm holes. Other M3 nuts sit in hex pockets in the deck.
UPRIGHT_UNDER_HEAD = 2.2             # 2.2 + 3.95 into the gearbox = 6.15
DECK_ABOVE_NUT = 1.5
# Checked on the test print: holes at 0, 60, ..., 300 deg (0 = forward, 90 = up),
# one each side and two above the shaft. The two below go unused: their screw
# heads would break into the shaft slot and sit 0.2 mm above the deck.
GEARBOX_HOLES_USED = (0, 60, 120, 180)

# --- On the lower deck ---
# Behind the motors: ~47 mm for the left motor's cable (it leaves backwards)
# and wiring, then strap slot, battery, strap slot, caster.
CASTER = (-90.0, 0.0, CASTER_FLANGE_LENGTH, CASTER_FLANGE_WIDTH)
# Its screws go in from the top, heads flush in 2.5 mm counterbores (one is under
# the battery), nuts under the flange.
BATTERY = (-80.8, 0.0, BATTERY_WIDTH, BATTERY_LENGTH)   # slides out sideways
# Two straps, one each side of the caster: under the deck they run at
# |y| 17-39, clear of the caster flange (|y| <= 16).
BATTERY_STRAP_SLOTS = [(x, y, 3.0, 22.0) for x in (-60.95, -100.65) for y in (28.0, -28.0)]
PERFBOARD = (59.0, 0.0, 50.0, 70.0)                     # 5 x 7 cm board
PERFBOARD_HOLES = (46.0, 65.5)       # measured centre spacing (x, y); holes Ø2.5
PILLARS = [(70.0, 70.0), (70.0, -70.0), (-45.0, 92.0), (-45.0, -92.0)]

# --- Bumper, on the lower deck's front edge ---
# Microswitches lie flat, 20 mm side along y, rollers facing forward. Their
# M2 holes are short slots along x, to set the rollers against the bumper.
MICROSWITCH_X = 93.25                # body centre
MICROSWITCH_HOLE_X = (MICROSWITCH_X + MICROSWITCH_WIDTH / 2
                      - MICROSWITCH_HOLES_TO_ROLLER_SIDE)
MICROSWITCH_Y = (35.0, -35.0)
MICROSWITCH_SLOT_LENGTH = 4.0
MICROSWITCH_NUT_CHANNEL = (4.3, 1.2)  # width, depth under the deck: M2 x 12 clears its nut
# The bumper hangs from two M3 pins that slide back in these slots.
BUMPER_GUIDE_SLOTS = [(97.0, y) for y in (55.0, -55.0)]
BUMPER_GUIDE_SLOT_LENGTH = 8.0

# --- On the upper deck ---
IMU = (0.0, 0.0, IMU_LENGTH, IMU_WIDTH)                 # above the wheel axis
CONVERTER = (-45.0, 70.0, CONVERTER_LENGTH + 2 * CONVERTER_TAB_LENGTH,
             CONVERTER_WIDTH)                           # tabs along x; powers the Pi
MDD3A = (50.0, 0.0, MDD3A_LENGTH, MDD3A_WIDTH)
PI = (-60.0, 0.0, PI_LENGTH, PI_WIDTH)               # USB and Ethernet face the rear
PI_HOLES = [(-17.5 - 3.5 - dx, dy) for dx in (0.0, 58.0) for dy in (24.5, -24.5)]
ROCKER_SWITCH = (90.0, -45.0)        # front-right, in reach
CABLE_SLOTS = [(15.0, y, 10.0, 30.0) for y in (45.0, -45.0)]

# --- LiDAR plate, on M3 standoffs above the Pi ---
LIDAR_X = -45.0
LIDAR_PLATE = (LIDAR_X, 0.0, 70.0, 70.0)
LIDAR_PLATE_HEIGHT = 28.0            # above the upper deck: 20 + 8 standoffs
LIDAR_PLATE_PILLARS = [(LIDAR_X + dx, dy) for dx in (-32.0, 32.0) for dy in (32.0, -32.0)]
BUMPER_MAX_MASS = 20.0               # grams; it sits furthest forward
