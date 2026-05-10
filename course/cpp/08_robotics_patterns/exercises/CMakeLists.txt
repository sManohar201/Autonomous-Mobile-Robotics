cmake_minimum_required(VERSION 3.16)
project(module08_robotics_patterns CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(ex01_observer ex01_observer.cpp)
add_executable(ex02_plugin_factory ex02_plugin_factory.cpp)
add_executable(ex03_fsm ex03_fsm.cpp)
add_executable(ex04_components ex04_components.cpp)
add_executable(ex05_capstone_localization ex05_capstone_localization.cpp)

add_executable(ex01_hard ex01_hard.cpp)
add_executable(ex02_hard ex02_hard.cpp)
add_executable(ex03_hard ex03_hard.cpp)
add_executable(ex04_hard ex04_hard.cpp)
add_executable(ex05_hard ex05_hard.cpp)
