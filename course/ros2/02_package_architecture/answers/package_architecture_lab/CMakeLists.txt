cmake_minimum_required(VERSION 3.16)
project(package_architecture_lab_answer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(package_boundary_review_answer src/package_boundary_review.cpp)
add_executable(dependency_audit_hard_answer src/dependency_audit_hard.cpp)
ament_target_dependencies(package_boundary_review_answer rclcpp std_msgs)
ament_target_dependencies(dependency_audit_hard_answer rclcpp std_msgs)

install(TARGETS package_boundary_review_answer dependency_audit_hard_answer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

