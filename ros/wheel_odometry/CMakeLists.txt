cmake_minimum_required(VERSION 3.16)
project(wheel_odometry)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
target_link_libraries(wheel_odometry_node PRIVATE
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS})

install(TARGETS wheel_odometry_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(diff_drive_odometry_test test/diff_drive_odometry_test.cpp)
endif()

ament_package()
