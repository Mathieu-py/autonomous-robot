# Laptop unit tests for the firmware's pure logic (no Pico SDK needed).
# Run: cmake -S firmware/tests -B firmware/tests/build
#      cmake --build firmware/tests/build && ctest --test-dir firmware/tests/build
cmake_minimum_required(VERSION 3.13)
project(firmware_tests CXX)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Werror)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../src)
enable_testing()

add_executable(safety_test safety_test.cpp ../src/safety.cpp)
add_test(NAME safety COMMAND safety_test)

add_executable(speed_controller_test speed_controller_test.cpp
  ../src/speed_controller.cpp)
add_test(NAME speed_controller COMMAND speed_controller_test)

add_executable(drive_command_test drive_command_test.cpp ../src/drive_command.cpp)
add_test(NAME drive_command COMMAND drive_command_test)

add_executable(phase_sequence_test phase_sequence_test.cpp ../src/phase_sequence.cpp)
add_test(NAME phase_sequence COMMAND phase_sequence_test)

add_executable(drivetrain_test drivetrain_test.cpp ../src/drivetrain.cpp
  ../src/wheel.cpp ../src/wheel_speed.cpp ../src/speed_controller.cpp
  ../src/safety.cpp)
add_test(NAME drivetrain COMMAND drivetrain_test)
