# 7.8 CMake Build Types

Debug builds are for debugging. Release builds are for performance measurement.

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

Do not compare performance numbers from Debug builds.
