cmake_minimum_required(VERSION 3.16)
project(sensor_fusion_exercises)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(ex01_ekf_2d ex01_ekf_2d.cpp)
target_link_libraries(ex01_ekf_2d Eigen3::Eigen)

add_executable(ex01_hard ex01_hard.cpp)
target_link_libraries(ex01_hard Eigen3::Eigen)
