# 3.1 Why Generic Programming Matters

Robotics code repeats the same shapes everywhere:

- Buffers of `ImuSample`, `LidarScan`, `Pose2d`, `WheelTick`
- Math functions that should work for `float`, `double`, and sometimes fixed-point numeric types
- Coordinate transforms over 2D and 3D vectors
- Drivers that differ by message type but share queueing, timestamp validation, and diagnostics

Copy-pasting the same implementation for each type is brittle. Runtime polymorphism solves some reuse problems, but it introduces indirection and often forces heap allocation. Templates solve a different problem: they let the compiler generate type-specific code from one source pattern.

```cpp
template <typename T>
T clamp(T value, T lo, T hi) {
    if (value < lo) return lo;
    if (hi < value) return hi;
    return value;
}

int main() {
    int pixels = clamp(300, 0, 255);
    double yaw = clamp(4.2, -3.14159, 3.14159);
}
```

The compiler instantiates `clamp<int>` and `clamp<double>` as needed. There is no virtual call and no runtime type check.

**Key idea:** templates are compile-time code generation with type checking.
