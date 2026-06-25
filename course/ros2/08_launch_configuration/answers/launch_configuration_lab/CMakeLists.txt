cmake_minimum_required(VERSION 3.8)
project(launch_configuration_lab_answer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(config_echo_node_answer src/config_echo_node.cpp)
ament_target_dependencies(config_echo_node_answer rclcpp std_msgs)

add_executable(variant_bringup_hard_answer src/variant_bringup_hard.cpp)
ament_target_dependencies(variant_bringup_hard_answer rclcpp std_msgs)

install(TARGETS
  config_echo_node_answer
  variant_bringup_hard_answer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
