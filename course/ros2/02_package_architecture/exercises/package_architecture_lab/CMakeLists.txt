cmake_minimum_required(VERSION 3.16)
project(package_architecture_lab)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(package_boundary_review src/package_boundary_review.cpp)
add_executable(dependency_audit_hard src/dependency_audit_hard.cpp)
ament_target_dependencies(package_boundary_review rclcpp std_msgs)
ament_target_dependencies(dependency_audit_hard rclcpp std_msgs)

install(TARGETS package_boundary_review dependency_audit_hard
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

