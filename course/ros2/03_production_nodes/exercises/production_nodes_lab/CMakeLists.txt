cmake_minimum_required(VERSION 3.16)
project(production_nodes_lab)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(parameter_validation_node src/parameter_validation_node.cpp)
add_executable(lifecycle_processor_hard src/lifecycle_processor_hard.cpp)
ament_target_dependencies(parameter_validation_node rclcpp std_msgs)
ament_target_dependencies(lifecycle_processor_hard rclcpp std_msgs)

install(TARGETS parameter_validation_node lifecycle_processor_hard
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

