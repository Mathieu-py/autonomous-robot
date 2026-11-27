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
                        IMU_WIDTH, LIDAR_ADAPTER_HOLE_INSET,
                        LIDAR_ADAPTER_LENGTH, LIDAR_ADAPTER_SOCKET_HOLE,
                        LIDAR_ADAPTER_SOCKET_PLUG,
                        LIDAR_ADAPTER_USB_HOLE, LIDAR_ADAPTER_USB_PLUG,
                        LIDAR_ADAPTER_WIDTH, LIDAR_HOLE_SPACING,
                        M3_SCREW_LENGTH, MDD3A_LENGTH, MDD3A_WIDTH,
                        MICROSWITCH_HOLES_TO_ROLLER_SIDE, MICROSWITCH_WIDTH,
                        MOTOR_LENGTH, PERFBOARD_HOLES_LENGTH,
                        PERFBOARD_HOLES_WIDTH, PERFBOARD_LENGTH,
                        PERFBOARD_WIDTH, PI_HEIGHT, PI_HOLE_INSET,
                        PI_HOLES_LENGTH, PI_HOLES_WIDTH, PI_LENGTH,
                        PI_FAR_USB_STACK, PI_USB_C_FROM_END,
                        PI_USB_C_PLUG, PI_WIDTH, LIDAR_SIZE,
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
ROCKER_SWITCH = (90.0, -45.0)        # front-right, in reach
# The centre slot takes the wires from below; the other, the converter's input.
CABLE_SLOTS = [(19.0, 0.0, 10.0, 40.0), (-55.0, 43.0, 30.0, 10.0)]

# The LiDAR plate's pillars were printed around this point; the LiDAR sits on it.
LIDAR_X = -45.0                      # mount with its cable side to the rear
LIDAR_Y = -10.0
LIDAR_PLATE_PILLARS = [(LIDAR_X + dx, LIDAR_Y + dy) for dx in (-32.0, 32.0) for dy in (32.0, -32.0)]

# Turned on 26 Nov (holes drilled by hand): between the front and rear
# pillars, its right end 48 mm past the right ones.
PI_X = LIDAR_X
PI_Y = LIDAR_Y - 32.0 - 48.0 + PI_LENGTH / 2
PI = (PI_X, PI_Y, PI_WIDTH, PI_LENGTH)               # USB-A to the left, USB-C forward
PI_FRONT_X = PI_X + PI_WIDTH / 2
PI_HOLE_END_Y = PI_Y - PI_LENGTH / 2
PI_PORTS_Y = PI_Y + PI_LENGTH / 2                    # USB-A and Ethernet, toward the converter
PI_HOLES = [(PI_X + sx * PI_HOLES_WIDTH / 2, PI_HOLE_END_Y + PI_HOLE_INSET + dy)
            for sx in (1, -1) for dy in (0.0, PI_HOLES_LENGTH)]
PI_STANDOFF = 6.0
PI_USB_C_Y = PI_HOLE_END_Y + PI_USB_C_FROM_END
PI_LIDAR_USB_X = PI_FRONT_X - PI_FAR_USB_STACK       # the rear stack; the Pico is in the middle one

# --- LiDAR plate, on M3 standoffs above the Pi ---
LIDAR_HOLES = [(LIDAR_X + sx * LIDAR_HOLE_SPACING / 2, LIDAR_Y + sy * LIDAR_HOLE_SPACING / 2)
               for sx in (1, -1) for sy in (1, -1)]
PILLAR_TO_PLATE_EDGE = 6.0           # room round each hole for its counterbore
LIDAR_PLATE_FRONT_X = LIDAR_X + 32.0 + PILLAR_TO_PLATE_EDGE
LIDAR_PLATE_BACK_X = LIDAR_X - 32.0 - PILLAR_TO_PLATE_EDGE
LIDAR_PLATE_LEFT_Y = 41.0            # extended left for the USB adapter
LIDAR_PLATE_RIGHT_Y = LIDAR_Y - 32.0 - PILLAR_TO_PLATE_EDGE
LIDAR_PLATE = ((LIDAR_PLATE_FRONT_X + LIDAR_PLATE_BACK_X) / 2,
               (LIDAR_PLATE_LEFT_Y + LIDAR_PLATE_RIGHT_Y) / 2,
               LIDAR_PLATE_FRONT_X - LIDAR_PLATE_BACK_X,
               LIDAR_PLATE_LEFT_Y - LIDAR_PLATE_RIGHT_Y)
LIDAR_PLATE_THICKNESS = 4.0
LIDAR_PLATE_CORNER_RADIUS = PILLAR_TO_PLATE_EDGE  # round the pillar holes, so an even wall
LIDAR_PLATE_HEIGHT = 35.0            # 11.5 mm of air above the Pi's fan
# (diameter, depth). From the top: an M3 x 6 grips 4 mm of the standoff, as on the upper deck.
PLATE_PILLAR_COUNTERBORE = (6.5, 2.0)
# From below: an M2.5 x 6 goes 3.5 mm into the LiDAR, under its 4 mm limit.
LIDAR_SCREW_COUNTERBORE = (5.5, 1.5)

# --- LiDAR's USB adapter, lengthwise along the LiDAR's left side ---
# Socket to the rear, near where the LiDAR's cable leaves; USB-C forward.
LIDAR_ADAPTER_STANDOFF = 6.0         # M2, as the IMU; a nut under the plate
LIDAR_ADAPTER_GAP = 1.0              # to the LiDAR's side
LIDAR_ADAPTER_Y = LIDAR_Y + LIDAR_SIZE / 2 + LIDAR_ADAPTER_GAP + LIDAR_ADAPTER_WIDTH / 2
# Its LiDAR plug ends flush with the LiDAR's back face.
LIDAR_ADAPTER_SOCKET_EDGE_X = LIDAR_X - LIDAR_SIZE / 2 + LIDAR_ADAPTER_SOCKET_PLUG
LIDAR_ADAPTER_USB_EDGE_X = LIDAR_ADAPTER_SOCKET_EDGE_X + LIDAR_ADAPTER_LENGTH
LIDAR_ADAPTER_X = LIDAR_ADAPTER_SOCKET_EDGE_X + LIDAR_ADAPTER_LENGTH / 2
LIDAR_ADAPTER = (LIDAR_ADAPTER_X, LIDAR_ADAPTER_Y, LIDAR_ADAPTER_LENGTH, LIDAR_ADAPTER_WIDTH)
# The socket end's hole is on the long edge nearer the LiDAR, the other on the outer edge.
LIDAR_ADAPTER_HOLES = [
    (LIDAR_ADAPTER_SOCKET_EDGE_X + LIDAR_ADAPTER_SOCKET_HOLE,
     LIDAR_ADAPTER_Y - LIDAR_ADAPTER_WIDTH / 2 + LIDAR_ADAPTER_HOLE_INSET),
    (LIDAR_ADAPTER_USB_EDGE_X - LIDAR_ADAPTER_USB_HOLE,
     LIDAR_ADAPTER_Y + LIDAR_ADAPTER_WIDTH / 2 - LIDAR_ADAPTER_HOLE_INSET)]
