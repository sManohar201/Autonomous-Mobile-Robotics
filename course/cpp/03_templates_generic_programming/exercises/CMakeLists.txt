cmake_minimum_required(VERSION 3.20)
project(module03_templates_generic_programming CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ex01_function_templates ex01_function_templates.cpp)
add_executable(ex02_static_vector ex02_static_vector.cpp)
add_executable(ex03_traits_concepts ex03_traits_concepts.cpp)
add_executable(ex04_crtp_units ex04_crtp_units.cpp)
add_executable(ex05_capstone_typed_ring_buffer ex05_capstone_typed_ring_buffer.cpp)

add_executable(ex01_hard ex01_hard.cpp)
add_executable(ex02_hard ex02_hard.cpp)
add_executable(ex03_hard ex03_hard.cpp)
add_executable(ex04_hard ex04_hard.cpp)
add_executable(ex05_hard ex05_hard.cpp)
