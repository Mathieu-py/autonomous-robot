cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type")

if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "Set PICO_SDK_PATH to your pico-sdk folder")
endif()
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(robot_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

pico_sdk_init()

add_executable(robot_firmware
  src/main.cpp
  src/motor.cpp
  src/encoder.cpp
  src/wheel_speed.cpp
  src/speed_controller.cpp
  src/test_sequences.cpp
)

# Encoder counting program from Raspberry Pi's pico-examples (BSD-3-Clause).
pico_generate_pio_header(robot_firmware ${CMAKE_CURRENT_LIST_DIR}/src/quadrature_encoder.pio)

# The Pico 2 W's LED is wired through its wireless chip, hence cyw43.
target_link_libraries(robot_firmware pico_stdlib hardware_pio hardware_pwm pico_cyw43_arch_none)

pico_enable_stdio_usb(robot_firmware 1)
pico_enable_stdio_uart(robot_firmware 0)

pico_add_extra_outputs(robot_firmware)
