cmake_minimum_required(VERSION 3.8)
project(gazebo_integration_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sim_time_check src/sim_time_check.cpp)
ament_target_dependencies(sim_time_check rclcpp std_msgs)

add_executable(gazebo_monitor_hard src/gazebo_monitor_hard.cpp)
ament_target_dependencies(gazebo_monitor_hard rclcpp rosgraph_msgs std_msgs)

install(TARGETS
  sim_time_check
  gazebo_monitor_hard
  DESTINATION lib/${PROJECT_NAME})

ament_package()
