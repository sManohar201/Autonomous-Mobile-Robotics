# 1.9 CMake Basics

```cmake
cmake_minimum_required(VERSION 3.15)
project(robot_math)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(mathutils src/mathutils.cpp)
target_include_directories(mathutils PUBLIC include)

add_executable(main_program src/main.cpp)
target_link_libraries(main_program PRIVATE mathutils)
```

Build:
```bash
cmake -B build
cmake --build build
./build/main_program
```

### `PUBLIC` vs `PRIVATE` vs `INTERFACE`

| Keyword | Meaning |
|---------|---------|
| `PRIVATE` | Only this target |
| `PUBLIC` | This target AND anything linking it |
| `INTERFACE` | Only things linking this target (not the target itself) |

If your library's headers are included by consumers, use `PUBLIC` on `target_include_directories`.

### Build types
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug      # -g, no optimisation
cmake -B build -DCMAKE_BUILD_TYPE=Release    # -O3, no debug symbols
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo  # -O2 + -g
```
