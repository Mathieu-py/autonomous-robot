"""Where each part sits, in millimetres.

Origin at the deck centre on the wheel axis, z = 0 on the floor, +x forward,
+y left. Parts are (centre x, centre y, size x, size y). Heavy parts sit
behind the axle so braking can't tip it forward (CoM 16 mm behind, 54 mm up).
"""
from dimensions import (BATTERY_LENGTH, BATTERY_WIDTH, CASTER_FLANGE_LENGTH,
                        CASTER_FLANGE_WIDTH, CASTER_HEIGHT, CONVERTER_LENGTH,
                        CONVERTER_TAB_LENGTH, CONVERTER_WIDTH,
                        GEARBOX_FACE_TO_WHEEL, GEARBOX_HOLE_CIRCLE,
                        GEARBOX_HOLE_DEPTH, GEARBOX_LENGTH, IMU_LENGTH,
                        IMU_WIDTH, M3_SCREW_LENGTH, MDD3A_LENGTH, MDD3A_WIDTH,
                        MICROSWITCH_HOLES_TO_ROLLER_SIDE, MICROSWITCH_WIDTH,
                        MOTOR_LENGTH, PERFBOARD_HOLES_LENGTH,
                        PERFBOARD_HOLES_WIDTH, PERFBOARD_LENGTH,
                        PERFBOARD_WIDTH, PI_HEIGHT, PI_HOLE_INSET,
                        PI_HOLES_LENGTH, PI_HOLES_WIDTH, PI_LENGTH,
                        PI_USB_C_FROM_FRONT, PI_USB_C_PLUG, PI_WIDTH,
                        SHAFT_OFFSET, WHEEL_DIAMETER, WHEEL_WIDTH)

# --- Drive geometry ---
ENCODER_GAP = 6.0                    # between the two motors' back ends
GEARBOX_FACE_Y = ENCODER_GAP / 2 + MOTOR_LENGTH
WHEEL_INNER_Y = GEARBOX_FACE_Y + GEARBOX_FACE_TO_WHEEL
WHEEL_CENTRE_Y = WHEEL_INNER_Y + WHEEL_WIDTH / 2
WHEEL_OUTER_Y = WHEEL_INNER_Y + WHEEL_WIDTH
WHEEL_SEPARATION = 2 * WHEEL_CENTRE_Y
AXLE_Z = WHEEL_DIAMETER / 2
GEARBOX_CENTRE_Z = AXLE_Z + SHAFT_OFFSET

# --- Decks ---
DECK_RADIUS = 120.0                  # leaves a ring outside the wheel slots
DECK_THICKNESS = 4.0
LOWER_DECK_BOTTOM = CASTER_HEIGHT    # the caster bolts straight on
LOWER_DECK_TOP = LOWER_DECK_BOTTOM + DECK_THICKNESS
PILLAR_STANDOFFS = (20.0, 20.0, 15.0)  # M3, stacked from the lower deck's top
UPPER_DECK_BOTTOM = LOWER_DECK_TOP + sum(PILLAR_STANDOFFS)  # 11 mm above the wheels
WHEEL_SLOT_HALF_LENGTH = 35.5        # along x; wheel chord through the deck plus margin
WHEEL_SLOT_MARGIN = 3.0              # each side of the tyre, along y
GEARBOX_CRADLE_DEPTH = 2.0           # pocket in the deck top under each gearbox

# --- Printed motor mounts (PETG), left side; the right mirrors it ---
MOUNT_THICKNESS = 5.0
MOUNT_ARCH_RADIUS = 23.0            # upright outline: an arch round the gearbox centre
MOUNT_FEET_X = (20.5, 32.0)          # |x| range of each foot, clear of the gearbox
MOUNT_FEET_Y = (34.0, GEARBOX_FACE_Y)
MOUNT_HOLES = [(sx * 26.25, y) for sx in (1, -1) for y in (39.0, 47.0)]  # M3 x 15
# The motor kit's M3 screws stop 0.05 mm short of the gearbox holes' bottom.
UPRIGHT_UNDER_HEAD = M3_SCREW_LENGTH - (GEARBOX_HOLE_DEPTH - 0.05)
DECK_ABOVE_NUT = 1.5
# Of 6 holes at 0-300 deg (0 forward, 90 up); the lower two hit the shaft slot.
GEARBOX_HOLES_USED = (0, 60, 120, 180)

# --- On the lower deck ---
# Behind the motors: room for the left motor's cable, then battery and caster.
BATTERY_X = -80.8
BATTERY = (BATTERY_X, 0.0, BATTERY_WIDTH, BATTERY_LENGTH)  # slides out sideways
# Screws from the top, heads in 2.5 mm counterbores, nuts under the flange.
CASTER = (-90.0, 0.0, CASTER_FLANGE_LENGTH, CASTER_FLANGE_WIDTH)
# One strap each side of the caster, clear of its flange under the deck.
STRAP_SLOT_SIZE = (3.0, 22.0)
STRAP_SLOT_GAP = 1.475               # battery side to each slot's near edge
STRAP_SLOT_X_OFFSET = BATTERY_WIDTH / 2 + STRAP_SLOT_GAP + STRAP_SLOT_SIZE[0] / 2
BATTERY_STRAP_SLOTS = [(BATTERY_X + sx * STRAP_SLOT_X_OFFSET, y, *STRAP_SLOT_SIZE)
                       for sx in (1, -1) for y in (28.0, -28.0)]
PERFBOARD = (59.0, 0.0, PERFBOARD_WIDTH, PERFBOARD_LENGTH)   # long side across
PERFBOARD_HOLES = (PERFBOARD_HOLES_WIDTH, PERFBOARD_HOLES_LENGTH)  # (x, y)
PILLARS = [(70.0, 70.0), (70.0, -70.0), (-45.0, 92.0), (-45.0, -92.0)]

# --- Bumper, on the lower deck's front edge ---
# Microswitches lie flat, rollers forward; slotted holes set them to the bumper.
MICROSWITCH_X = 93.25                # body centre
MICROSWITCH_HOLE_X = (MICROSWITCH_X + MICROSWITCH_WIDTH / 2
                      - MICROSWITCH_HOLES_TO_ROLLER_SIDE)
MICROSWITCH_Y = (35.0, -35.0)
MICROSWITCH_SLOT_LENGTH = 4.0
MICROSWITCH_NUT_CHANNEL = (4.3, 1.2)  # width, depth under the deck
# The bumper hangs from two M3 pins that slide back in these slots.
BUMPER_GUIDE_SLOTS = [(97.0, y) for y in (55.0, -55.0)]
BUMPER_GUIDE_SLOT_LENGTH = 8.0
BUMPER_MAX_MASS_G = 20.0             # it sits furthest forward

# --- On the upper deck ---
IMU = (0.0, 0.0, IMU_LENGTH, IMU_WIDTH)                 # above the wheel axis
CONVERTER = (-45.0, 70.0, CONVERTER_LENGTH + 2 * CONVERTER_TAB_LENGTH,
             CONVERTER_WIDTH)                           # tabs along x; powers the Pi
MDD3A = (50.0, 0.0, MDD3A_LENGTH, MDD3A_WIDTH)
# 10 mm right of centre, so its USB-C plug can bend before the converter.
PI_X = -60.0
PI_Y = -10.0
PI = (PI_X, PI_Y, PI_LENGTH, PI_WIDTH)               # USB-A to the rear, USB-C left
PI_FRONT_X = PI_X + PI_LENGTH / 2
PI_HOLES = [(PI_FRONT_X - PI_HOLE_INSET - dx, PI_Y + sy * PI_HOLES_WIDTH / 2)
            for dx in (0.0, PI_HOLES_LENGTH) for sy in (1, -1)]
PI_STANDOFF = 6.0
PI_USB_C_X = PI_FRONT_X - PI_USB_C_FROM_FRONT
ROCKER_SWITCH = (90.0, -45.0)        # front-right, in reach
# The centre slot takes the wires from below; the other, the converter's input.
CABLE_SLOTS = [(19.0, 0.0, 10.0, 40.0), (-55.0, 43.0, 30.0, 10.0)]

# --- LiDAR plate, on M3 standoffs above the Pi ---
LIDAR_X = -45.0                      # mount with its cable side to the rear
LIDAR_PLATE = (LIDAR_X, PI_Y, 70.0, 70.0)
LIDAR_PLATE_THICKNESS = 4.0
LIDAR_PLATE_HEIGHT = 35.0            # 11.5 mm of air above the Pi's fan
LIDAR_PLATE_PILLARS = [(LIDAR_X + dx, PI_Y + dy) for dx in (-32.0, 32.0) for dy in (32.0, -32.0)]
