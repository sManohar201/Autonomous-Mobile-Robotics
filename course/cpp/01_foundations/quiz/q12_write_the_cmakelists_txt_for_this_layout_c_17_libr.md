# **Q12.** Write the CMakeLists.txt for this layout. C++17, library named `filters_lib`, executable named `robot_filter`. Both library sources and `main.cpp` need `include/`.

```
project/
├── CMakeLists.txt
├── include/filters.hpp
└── src/
    ├── filters.cpp
    ├── ekf.cpp
    └── main.cpp
```

<details>
<summary>Answer</summary>

```cmake
cmake_minimum_required(VERSION 3.15)
project(robot_filter)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(filters_lib src/filters.cpp src/ekf.cpp)
target_include_directories(filters_lib PUBLIC include)

add_executable(robot_filter src/main.cpp)
target_link_libraries(robot_filter PRIVATE filters_lib)
```

</details>
