cmake_minimum_required(VERSION 3.8)
project(observability_operations_lab_answer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(diagnostic_aggregator_answer src/diagnostic_aggregator.cpp)
ament_target_dependencies(diagnostic_aggregator_answer rclcpp std_msgs)

add_executable(fault_report_hard_answer src/fault_report_hard.cpp)
ament_target_dependencies(fault_report_hard_answer rclcpp std_msgs)

install(TARGETS
  diagnostic_aggregator_answer
  fault_report_hard_answer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
