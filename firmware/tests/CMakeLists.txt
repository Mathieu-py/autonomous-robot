# Laptop unit tests for the firmware's pure logic (no Pico SDK needed).
# Run: cmake -S firmware/tests -B firmware/tests/build
#      cmake --build firmware/tests/build && ctest --test-dir firmware/tests/build
cmake_minimum_required(VERSION 3.13)
project(firmware_tests CXX)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Werror)
# Trap undefined behaviour, such as a NaN cast to an integer.
add_compile_options(-fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all)
add_link_options(-fsanitize=undefined,float-cast-overflow)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../src)
# The protocol examples shared with the bridge's tests.
include_directories(${CMAKE_CURRENT_LIST_DIR}/../../ros/pico_bridge/test)

find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

add_executable(firmware_tests
  drive_command_test.cpp
  drivetrain_test.cpp
  imu_reading_test.cpp
  periodic_timer_test.cpp
  phase_sequence_test.cpp
  pi_lines_test.cpp
  pwm_levels_test.cpp
  safety_test.cpp
  speed_controller_test.cpp
  ../src/bench/phase_sequence.cpp
  ../src/devices/imu_reading.cpp
  ../src/devices/pwm_levels.cpp
  ../src/drive/drive_command.cpp
  ../src/drive/drivetrain.cpp
  ../src/drive/safety.cpp
  ../src/drive/speed_controller.cpp
  ../src/drive/wheel.cpp
  ../src/pi_lines.cpp)
target_link_libraries(firmware_tests GTest::gtest_main)
gtest_discover_tests(firmware_tests)
