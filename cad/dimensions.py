"""Every dimension the chassis models use, in millimetres.

Measured: caliper measurements of our parts (2026-11-10).
Datasheet: the manufacturer's drawing, linked next to the value.
"""

# --- Motor (JGB37-520), measured ---
GEARBOX_DIAMETER = 36.95
MOTOR_BODY_DIAMETER = 32.8           # motor and encoder behind the gearbox
MOTOR_LENGTH = 56.75                 # gearbox front face to the encoder's back
GEARBOX_FACE_TO_WHEEL = 16.7         # gearbox front face to the wheel's inner face

# --- Wheel, measured ---
WHEEL_DIAMETER = 68.0
WHEEL_WIDTH = 27.0

# --- Motor bracket, measured, with the motor hanging under the deck ---
BRACKET_BASE_ACROSS = 40.32          # parallel to the wheel
BRACKET_BASE_ALONG = 42.9            # along the motor, from the upright inwards
BRACKET_HOLE_DIAMETER = 3.5
BRACKET_HOLES_ACROSS = 30.0          # centred across the base
BRACKET_HOLES_ALONG = 23.8
BRACKET_UPRIGHT_TO_HOLES = 10.0      # upright's inner face to the nearest holes
BRACKET_BASE_TO_SHAFT = 34.0         # shaft sits low, gearbox up against the base
BRACKET_BASE_TO_LOWEST = 48.0        # the upright reaches ~2 mm below the gearbox
BRACKET_BASE_TO_GEARBOX_BOTTOM = 45.6

# --- Battery (Zeee 3S 2200 mAh), measured ---
BATTERY_LENGTH = 77.0
BATTERY_WIDTH = 33.75
BATTERY_HEIGHT = 27.3                # leads leave the 33.75 x 27.3 end

# --- 5 V converter, measured ---
CONVERTER_LENGTH = 46.0
CONVERTER_WIDTH = 31.5
CONVERTER_HEIGHT = 18.1              # cables leave one 46 x 18.1 side
CONVERTER_TAB_LENGTH = 9.5           # tabs on both ends, flush with the bottom
CONVERTER_SLOT_WIDTH = 4.5           # open-ended slot in each tab
CONVERTER_SLOT_SPACING = 50.5        # between the slots' round ends

# --- Ball caster (5/8" ball transfer unit), measured ---
CASTER_HEIGHT = 20.0                 # mounting face to the bottom of the ball
CASTER_FLANGE_LENGTH = 49.0          # rounded ends
CASTER_FLANGE_WIDTH = 32.0
CASTER_HOLE_DIAMETER = 4.0
CASTER_HOLE_SPACING = 38.0           # measured 37-38; listings say 39
CASTER_BALL_OPENING = 14.67
CASTER_FLANGE_THICKNESS = 1.0        # at the screw holes: M3 x 6 from the top fits

# --- Rocker switch (KCD1-style), measured ---
SWITCH_BEZEL_LENGTH = 20.8
SWITCH_BEZEL_WIDTH = 14.85
SWITCH_BODY_LENGTH = 17.25           # next to the bezel
SWITCH_BODY_WIDTH = 11.75
SWITCH_CLIP_SPAN = 18.75             # across the snap clips
SWITCH_CLIP_GAP = 3.0                # bezel underside to the clips' catching edge
SWITCH_CUTOUT_LENGTH = 17.6          # clears the body, squeezes the clips
SWITCH_CUTOUT_WIDTH = 12.1
SWITCH_PANEL_THICKNESS = 2.5         # deck thinned to this around the switch

# --- RPLIDAR C1, datasheet (Slamtec C1 datasheet v1.0, figure 4-1) ---
LIDAR_SIZE = 55.6                    # square base
LIDAR_HEIGHT = 41.3
LIDAR_BASE_HEIGHT = 23.1
LIDAR_SCAN_HEIGHT = 29.8             # laser plane above the bottom
LIDAR_HOLE_SPACING = 43.0            # 4 x M2.5, square
LIDAR_SCREW_MAX_DEPTH = 4.0

# --- Cytron MDD3A, datasheet (figure 2) ---
MDD3A_LENGTH = 48.26
MDD3A_WIDTH = 41.66
MDD3A_HOLES_LENGTH = 43.18
MDD3A_HOLES_WIDTH = 36.58
MDD3A_HOLE_DIAMETER = 3.0

# --- Raspberry Pi 5, datasheet (mechanical drawing) ---
PI_LENGTH = 85.0
PI_WIDTH = 56.0
PI_HOLES_LENGTH = 58.0
PI_HOLES_WIDTH = 49.0
PI_HOLE_INSET = 3.5                  # from the corner, on both edges
PI_HOLE_DIAMETER = 2.7               # M2.5

# --- Raspberry Pi Pico 2 W, datasheet ---
PICO_LENGTH = 51.0
PICO_WIDTH = 21.0
PICO_HOLES_LENGTH = 47.0
PICO_HOLES_WIDTH = 11.4
PICO_HOLE_DIAMETER = 2.1

# --- Adafruit MPU-6050 (#3886), datasheet size, measured holes ---
IMU_LENGTH = 26.0
IMU_WIDTH = 17.8
IMU_HOLES_LENGTH = 20.32             # measured ~20.25; Adafruit's 0.8 in grid
IMU_HOLES_WIDTH = 12.7               # measured ~12.5; Adafruit's 0.5 in grid
IMU_HOLE_DIAMETER = 2.5

# --- KW12-3 roller-lever microswitch, measured ---
MICROSWITCH_LENGTH = 20.0
MICROSWITCH_WIDTH = 10.5
MICROSWITCH_THICKNESS = 6.5
MICROSWITCH_HOLE_SPACING = 9.5
MICROSWITCH_HOLE_DIAMETER = 2.3      # M2
MICROSWITCH_ROLLER_HEIGHT = 11.0     # roller top above the body, at rest
MICROSWITCH_HOLES_TO_ROLLER_SIDE = 6.8
MICROSWITCH_TRAVEL_TO_CLICK = 3.0

# --- Screws on hand (motor kits), measured ---
M3_SCREW_LENGTH = 6.15               # under the head; 7.8 overall
M3_HEAD_CLEARANCE = 6.5              # counterbore diameter
M3_NUT_ACROSS_FLATS = 5.8            # pocket, with clearance for a 5.5 nut
M3_NUT_THICKNESS = 2.4
GEARBOX_HOLE_DEPTH = 4.0             # listings

# --- Nylon standoffs (male-female: body + 6 mm thread), from the kit ---
STANDOFF_LENGTHS = (6.0, 8.0, 12.0, 15.0, 20.0)  # M2.5 and M3 alike; they stack

# --- Printing ---
PRINT_BED = 256.0                    # UW Bambu P1S/P2S
