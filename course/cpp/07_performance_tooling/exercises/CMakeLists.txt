cmake_minimum_required(VERSION 3.16)
project(module07_performance_tooling CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ex01_benchmark_timer ex01_benchmark_timer.cpp)
add_executable(ex02_allocation ex02_allocation.cpp)
add_executable(ex03_cache_layout ex03_cache_layout.cpp)
add_executable(ex04_sanitizer_bughunt ex04_sanitizer_bughunt.cpp)
add_executable(ex05_capstone_pointcloud_transform ex05_capstone_pointcloud_transform.cpp)

add_executable(ex01_hard ex01_hard.cpp)
add_executable(ex02_hard ex02_hard.cpp)
add_executable(ex03_hard ex03_hard.cpp)
add_executable(ex04_hard ex04_hard.cpp)
add_executable(ex05_hard ex05_hard.cpp)
