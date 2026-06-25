cmake_minimum_required(VERSION 3.16)
project(production_nodes_lab_answer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(parameter_validation_node_answer src/parameter_validation_node.cpp)
add_executable(lifecycle_processor_hard_answer src/lifecycle_processor_hard.cpp)
ament_target_dependencies(parameter_validation_node_answer rclcpp std_msgs)
ament_target_dependencies(lifecycle_processor_hard_answer rclcpp std_msgs)

install(TARGETS parameter_validation_node_answer lifecycle_processor_hard_answer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

