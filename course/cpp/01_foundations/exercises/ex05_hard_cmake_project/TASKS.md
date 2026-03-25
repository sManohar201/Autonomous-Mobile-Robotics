# Exercise 05 (Hard) — Production CMake: Multi-Library Dependency Graph,
#                       Sanitizers, and Install

## Context

Every ROS2 package is a CMake project. `ament_cmake` is CMake with a thin
layer on top. A production robotics CMakeLists.txt:

- Correctly propagates include directories **transitively** across libraries
- Knows precisely when to use PUBLIC vs PRIVATE vs INTERFACE
- Enables AddressSanitizer and UBSan for debug builds
- Generates `compile_commands.json` for clangd / clang-tidy
- Provides install targets so other CMake projects can consume the library

**This exercise requires you to write the entire CMakeLists.txt from scratch.**
The source files (vec3.hpp/cpp, lowpass.hpp/cpp, main.cpp) are already
provided and correct. Your only deliverable is CMakeLists.txt.

---

## Pre-coding research questions

Answer each question as a comment block at the **top** of your CMakeLists.txt
before any CMake code. These answers are part of the exercise.

**Q1.** What is the difference between PUBLIC, PRIVATE, and INTERFACE in
`target_link_libraries` and `target_include_directories`?

**Q2.** `sensor_utils` links `robot_math` as PRIVATE, and
`sensor_utils/lowpass.hpp` includes `<robot_math/vec3.hpp>`. What happens
when `robot_app` tries to `#include <sensor_utils/lowpass.hpp>` and compile?
Why does this fail, and which keyword fixes it?

**Q3.** What is a CMake INTERFACE library? Give a concrete case where you
would create a library target with no source files.

**Q4.** How do you enable AddressSanitizer for a CMake target? Which flags
must appear on BOTH compile AND link steps, and why?

**Q5.** What does `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` generate? Which
developer tools consume this file?

**Q6.** What does `cmake --install build --prefix /tmp/robot_install` do?
What CMake commands must your CMakeLists.txt contain for this to work?

---

## Project structure

```
ex05_hard_cmake_project/
  CMakeLists.txt               ← YOU WRITE THIS (the only file you touch)
  libs/
    robot_math/
      include/robot_math/
        vec3.hpp
      src/
        vec3.cpp
    sensor_utils/
      include/sensor_utils/
        lowpass.hpp            ← #includes <robot_math/vec3.hpp>
      src/
        lowpass.cpp
  app/
    src/
      main.cpp                 ← #includes both headers
```

---

## Requirements for CMakeLists.txt

### R1: Basic setup
- `cmake_minimum_required(VERSION 3.16)`
- `project(robot_system LANGUAGES CXX)`
- C++17 standard, required (use `CMAKE_CXX_STANDARD` and `CMAKE_CXX_STANDARD_REQUIRED`)

### R2: robot_math static library
- Sources: `libs/robot_math/src/vec3.cpp`
- Include directories: `libs/robot_math/include`
  — Use **PUBLIC** so that any target linking robot_math gets these include dirs automatically.

### R3: sensor_utils static library
- Sources: `libs/sensor_utils/src/lowpass.cpp`
- Its own include dirs: `libs/sensor_utils/include` — **PUBLIC**
- Links robot_math — **PUBLIC or PRIVATE?**

  `sensor_utils/lowpass.hpp` **includes** `<robot_math/vec3.hpp>`.
  Any code that `#include <sensor_utils/lowpass.hpp>` also needs robot_math's
  include path to resolve that nested include.

  **Which keyword ensures robot_math's include directories propagate to
  sensor_utils's consumers?** Write your answer as a comment in CMakeLists.txt.

### R4: robot_app executable
- Sources: `app/src/main.cpp`
- Links sensor_utils
- **Does robot_app need to explicitly link robot_math?**
  Verify by checking whether it compiles without it (given your R3 keyword
  choice). Write your answer as a comment.

### R5: Compile options
- Add `-Wall -Wextra` to ALL three targets
- Use `target_compile_options(...)`
- THINK: Use PRIVATE (warnings are a build-time concern, not a consumer API)

### R6: Optional AddressSanitizer
- Add CMake cache variable: `option(WITH_ASAN "Enable AddressSanitizer" OFF)`
- If `WITH_ASAN` is ON, add `-fsanitize=address -fno-omit-frame-pointer` to
  compile options AND `-fsanitize=address` to link options for ALL targets.

  Test:
  ```bash
  cmake -B build -DCMAKE_BUILD_TYPE=Debug -DWITH_ASAN=ON
  cmake --build build
  ./build/robot_app
  ```

### R7: Compile commands
Add this comment to your CMakeLists.txt explaining how to generate
`compile_commands.json`:

```cmake
# Generate compile_commands.json (used by clangd, clang-tidy, ccls):
#   cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
# Then symlink into the project root:
#   ln -sf build/compile_commands.json compile_commands.json
```

### R8: Install targets (research required — bonus)
- `install(TARGETS robot_math sensor_utils robot_app ...)`
  with ARCHIVE, LIBRARY, RUNTIME destinations set to `lib`, `lib`, `bin`
- `install(DIRECTORY libs/robot_math/include/ libs/sensor_utils/include/
           DESTINATION include)`
- Test:
  ```bash
  cmake --install build --prefix /tmp/robot_install
  ls /tmp/robot_install/include/robot_math/
  ls /tmp/robot_install/bin/
  ```

---

## Build and test

```bash
# Basic build
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
./build/robot_app
```

Expected output:
```
gravity = (0, 0, 9.81)
filtered = (0, 0, 1.05)
```

With sanitizers:
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug -DWITH_ASAN=ON
cmake --build build
./build/robot_app
```

With compile commands:
```bash
cmake -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -sf build/compile_commands.json compile_commands.json
# Now open the project in an editor with clangd — you get full intellisense.
```

---

## Key insight

`sensor_utils/lowpass.hpp` includes `<robot_math/vec3.hpp>`.
This means **every consumer of sensor_utils also needs robot_math's include
path** — even if they never directly use robot_math.

The correct CMake keyword to ensure this propagation is: **________**.

Fill in your answer as a comment in CMakeLists.txt.

---

## What a correct CMakeLists.txt looks like (structure — NOT the answer)

```cmake
# [Answers to Q1–Q6 as comments]
cmake_minimum_required(...)
project(...)
set(CMAKE_CXX_STANDARD ...)
...
add_library(robot_math STATIC ...)
target_include_directories(robot_math ...)
target_compile_options(robot_math ...)

add_library(sensor_utils STATIC ...)
target_include_directories(sensor_utils ...)
target_link_libraries(sensor_utils ... robot_math)
target_compile_options(sensor_utils ...)

add_executable(robot_app ...)
target_link_libraries(robot_app ... sensor_utils)
target_compile_options(robot_app ...)

option(WITH_ASAN ...)
if(WITH_ASAN)
    foreach(tgt robot_math sensor_utils robot_app)
        target_compile_options(${tgt} ...)
        target_link_options(${tgt} ...)
    endforeach()
endif()

install(TARGETS ...)
install(DIRECTORY ...)
```
