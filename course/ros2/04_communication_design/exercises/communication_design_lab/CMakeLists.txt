cmake_minimum_required(VERSION 3.16)
project(communication_design_lab)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(qos_status_publisher src/qos_status_publisher.cpp)
add_executable(command_api_hard src/command_api_hard.cpp)
ament_target_dependencies(qos_status_publisher rclcpp std_msgs std_srvs)
ament_target_dependencies(command_api_hard rclcpp std_msgs std_srvs)
install(TARGETS qos_status_publisher command_api_hard DESTINATION lib/${PROJECT_NAME})
ament_package()

