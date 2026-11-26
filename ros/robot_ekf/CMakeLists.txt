cmake_minimum_required(VERSION 3.16)
project(robot_ekf)

set(CMAKE_CXX_STANDARD 20)
add_compile_options(-Wall -Wextra -Wpedantic)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(robot_ekf_node src/ekf_node.cpp)
target_link_libraries(robot_ekf_node PRIVATE
  Eigen3::Eigen
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${builtin_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS})

install(TARGETS robot_ekf_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ekf_test test/ekf_test.cpp)
  target_link_libraries(ekf_test Eigen3::Eigen)
  ament_add_gtest(reorder_buffer_test test/reorder_buffer_test.cpp)
endif()

ament_package()
