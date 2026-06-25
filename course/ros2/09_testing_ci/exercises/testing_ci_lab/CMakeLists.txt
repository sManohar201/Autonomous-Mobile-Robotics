cmake_minimum_required(VERSION 3.8)
project(testing_ci_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(freshness_logic_node src/freshness_logic_node.cpp)
ament_target_dependencies(freshness_logic_node rclcpp std_msgs)

add_executable(test_report_hard src/test_report_hard.cpp)
ament_target_dependencies(test_report_hard rclcpp std_msgs)

install(TARGETS
  freshness_logic_node
  test_report_hard
  DESTINATION lib/${PROJECT_NAME})

ament_package()
