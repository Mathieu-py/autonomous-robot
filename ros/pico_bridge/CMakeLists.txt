cmake_minimum_required(VERSION 3.16)
project(pico_bridge)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pico_bridge_node src/pico_bridge_node.cpp src/serial_port.cpp)
target_include_directories(pico_bridge_node PRIVATE include)
target_link_libraries(pico_bridge_node PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(TARGETS pico_bridge_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(drive_protocol_test test/drive_protocol_test.cpp)
  target_include_directories(drive_protocol_test PRIVATE include)
endif()

ament_package()
