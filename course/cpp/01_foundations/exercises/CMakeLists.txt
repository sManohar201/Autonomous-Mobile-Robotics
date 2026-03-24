cmake_minimum_required(VERSION 3.15)
project(module01_foundations)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Exercises 1-4: single-file executables
add_executable(ex01_pipeline   ex01_pipeline.cpp)
add_executable(ex02_types      ex02_types_const.cpp)
add_executable(ex03_refs       ex03_pointers_refs.cpp)
add_executable(ex04_namespaces ex04_namespaces.cpp)

# Exercise 1 task 3: hypotenuse as a separate library
add_library(hypotenuse hypotenuse.cpp)
target_link_libraries(ex01_pipeline PRIVATE hypotenuse)

# Exercise 5: multi-file library project
add_library(vector2d ex05_cmake_project/src/vector2d.cpp)
target_include_directories(vector2d PUBLIC ex05_cmake_project/include)

add_executable(ex05_cmake ex05_cmake_project/src/main.cpp)
target_link_libraries(ex05_cmake PRIVATE vector2d)
