cmake_minimum_required(VERSION 3.16)
project(module05_standard_library CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ex01_containers ex01_containers.cpp)
add_executable(ex02_algorithms ex02_algorithms.cpp)
add_executable(ex03_chrono ex03_chrono.cpp)
add_executable(ex04_string_view ex04_string_view.cpp)
add_executable(ex05_capstone_sensor_window ex05_capstone_sensor_window.cpp)

add_executable(ex01_hard ex01_hard.cpp)
add_executable(ex02_hard ex02_hard.cpp)
add_executable(ex03_hard ex03_hard.cpp)
add_executable(ex04_hard ex04_hard.cpp)
add_executable(ex05_hard ex05_hard.cpp)
