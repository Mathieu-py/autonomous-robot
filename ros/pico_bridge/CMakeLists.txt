cmake_minimum_required(VERSION 3.16)
project(pico_bridge)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pico_bridge_node src/pico_bridge_node.cpp src/serial_port.cpp)
target_link_libraries(pico_bridge_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pico_protocol_test test/pico_protocol_test.cpp)
  target_include_directories(pico_protocol_test PRIVATE test)
  ament_add_gtest(drive_kinematics_test test/drive_kinematics_test.cpp)
  ament_add_gtest(gyro_bias_test test/gyro_bias_test.cpp)
endif()

ament_package()
