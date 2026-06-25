cmake_minimum_required(VERSION 3.16)
project(robot_status_publisher_answer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(robot_status_publisher_answer src/robot_status_publisher.cpp)
ament_target_dependencies(robot_status_publisher_answer rclcpp std_msgs std_srvs)

install(TARGETS robot_status_publisher_answer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

