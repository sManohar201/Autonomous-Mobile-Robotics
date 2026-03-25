cmake_minimum_required(VERSION 3.16)
project(module02_exercises CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Standard exercises ────────────────────────────────────────────────────────
add_executable(ex01_constructors ex01_constructors.cpp)
add_executable(ex02_copy_move    ex02_copy_move.cpp)
add_executable(ex03_raii         ex03_raii.cpp)
add_executable(ex04_operators    ex04_operators.cpp)
add_executable(ex05_inheritance  ex05_inheritance.cpp)

# ── Hard exercises ────────────────────────────────────────────────────────────
add_executable(ex01_hard  ex01_constructors_hard.cpp)
add_executable(ex02_hard  ex02_copy_move_hard.cpp)
add_executable(ex03_hard  ex03_raii_hard.cpp)
add_executable(ex04_hard  ex04_operators_hard.cpp)
add_executable(ex05_hard  ex05_inheritance_hard.cpp)

# ── Capstone: 1D Kalman Filter (hard) ────────────────────────────────────────
add_library(kalman1d ex06_kalman1d/src/kalman1d.cpp)
target_include_directories(kalman1d PUBLIC ex06_kalman1d/include)

add_executable(ex06_kalman1d_main ex06_kalman1d/src/main.cpp)
target_link_libraries(ex06_kalman1d_main PRIVATE kalman1d)
