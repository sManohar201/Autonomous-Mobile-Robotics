cmake_minimum_required(VERSION 3.16)
project(robot_monitor_prototype_answer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(robot_monitor_prototype_answer src/robot_monitor_prototype.cpp)
ament_target_dependencies(robot_monitor_prototype_answer rclcpp std_msgs)

install(TARGETS robot_monitor_prototype_answer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

