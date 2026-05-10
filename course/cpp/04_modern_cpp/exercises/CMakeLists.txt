cmake_minimum_required(VERSION 3.16)
project(module04_modern_cpp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ex01_auto_structured_bindings ex01_auto_structured_bindings.cpp)
add_executable(ex02_lambdas_algorithms ex02_lambdas_algorithms.cpp)
add_executable(ex03_optional_variant ex03_optional_variant.cpp)
add_executable(ex04_constexpr_folds ex04_constexpr_folds.cpp)
add_executable(ex05_capstone_config_state_machine ex05_capstone_config_state_machine.cpp)

add_executable(ex01_hard ex01_hard.cpp)
add_executable(ex02_hard ex02_hard.cpp)
add_executable(ex03_hard ex03_hard.cpp)
add_executable(ex04_hard ex04_hard.cpp)
add_executable(ex05_hard ex05_hard.cpp)
