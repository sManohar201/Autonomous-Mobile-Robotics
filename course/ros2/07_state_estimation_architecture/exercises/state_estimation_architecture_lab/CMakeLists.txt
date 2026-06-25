cmake_minimum_required(VERSION 3.8)
project(state_estimation_architecture_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(measurement_adapter src/measurement_adapter.cpp)
ament_target_dependencies(measurement_adapter nav_msgs rclcpp std_msgs)

add_executable(localization_shell_hard src/localization_shell_hard.cpp)
ament_target_dependencies(localization_shell_hard nav_msgs rclcpp std_msgs std_srvs)

install(TARGETS
  measurement_adapter
  localization_shell_hard
  DESTINATION lib/${PROJECT_NAME})

ament_package()
