# **Q10.** This CMakeLists.txt fails to build. Find and fix the mistake.

```cmake
add_library(sensor src/sensor.cpp)
add_executable(demo src/main.cpp)
target_include_directories(sensor PRIVATE include)
target_link_libraries(demo PRIVATE sensor)
```

`main.cpp` includes `"sensor.hpp"` from `include/` but the build fails with "file not found".

<details>
<summary>Answer</summary>

`PRIVATE` means the include path is only available when compiling `sensor.cpp` — it is not propagated to `demo`. Fix: change to `PUBLIC`.

```cmake
target_include_directories(sensor PUBLIC include)
```

Rule: if your library's headers are part of its public API (included by consumers), use `PUBLIC`.

</details>
