cmake_minimum_required(VERSION 3.8)
project(field_deployable_stack_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(deployment_checklist_node src/deployment_checklist_node.cpp)
ament_target_dependencies(deployment_checklist_node rclcpp std_msgs)

add_executable(degraded_mode_manager_hard src/degraded_mode_manager_hard.cpp)
ament_target_dependencies(degraded_mode_manager_hard rclcpp std_msgs)

install(TARGETS
  deployment_checklist_node
  degraded_mode_manager_hard
  DESTINATION lib/${PROJECT_NAME})

ament_package()
