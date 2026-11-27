cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "Set PICO_SDK_PATH to your pico-sdk folder")
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
# Built and run on 2.3.1; the stdout timeout and pico2_w rely on recent SDKs.
if(PICO_SDK_VERSION_STRING VERSION_LESS "2.3.1")
  message(FATAL_ERROR "Needs pico-sdk 2.3.1 or newer, found ${PICO_SDK_VERSION_STRING}")
endif()

project(robot_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

set(FIRMWARE_SOURCES
  src/main.cpp
  src/pi_lines.cpp
  src/bench/imu_test.cpp
  src/bench/mode.cpp
  src/bench/motor_tests.cpp
  src/bench/phase_sequence.cpp
  src/devices/encoder.cpp
  src/devices/imu.cpp
  src/devices/imu_reading.cpp
  src/devices/motor.cpp
  src/devices/pwm_levels.cpp
  src/drive/drive_command.cpp
  src/drive/drivetrain.cpp
  src/drive/safety.cpp
  src/drive/speed_controller.cpp
  src/drive/wheel.cpp
)
add_executable(robot_firmware ${FIRMWARE_SOURCES})
# Warnings on our sources only: the SDK compiles its own into this target too.
set_source_files_properties(${FIRMWARE_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")
target_include_directories(robot_firmware PRIVATE src)

# Encoder counting program from Raspberry Pi's pico-examples (BSD-3-Clause).
pico_generate_pio_header(robot_firmware ${CMAKE_CURRENT_LIST_DIR}/src/devices/quadrature_encoder.pio)

# The Pico 2 W's LED is wired through its wireless chip, hence cyw43.
target_link_libraries(robot_firmware pico_stdlib hardware_adc hardware_i2c hardware_pio
  hardware_pwm hardware_watchdog pico_cyw43_arch_none)

# Ticks per wheel turn, from the one copy shared with ROS; edits re-run CMake.
set(ROBOT_YAML ${CMAKE_CURRENT_LIST_DIR}/../ros/robot_description/config/robot.yaml)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${ROBOT_YAML})
file(STRINGS ${ROBOT_YAML} TICKS_LINE REGEX "^ *ticks_per_rev:")
string(REGEX MATCH "[0-9]+(\\.[0-9]+)?" TICKS_PER_REV "${TICKS_LINE}")
if(NOT TICKS_PER_REV)
  message(FATAL_ERROR "No ticks_per_rev in ${ROBOT_YAML}")
endif()
target_compile_definitions(robot_firmware PRIVATE ROBOT_TICKS_PER_REV=${TICKS_PER_REV})

# Drop unread serial output instead of blocking the loop (and the watchdog).
target_compile_definitions(robot_firmware PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=0)
# Room for a period's lines (up to ~160 bytes); what doesn't fit is dropped.
target_compile_definitions(robot_firmware PRIVATE CFG_TUD_CDC_TX_BUFSIZE=256)

pico_enable_stdio_usb(robot_firmware 1)
pico_enable_stdio_uart(robot_firmware 0)

pico_add_extra_outputs(robot_firmware)
