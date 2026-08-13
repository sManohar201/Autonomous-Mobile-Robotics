# 4.10 Overload Pattern for Variant Visitors

The overload pattern combines multiple lambdas into one visitor.

```cpp
template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;
```

Usage:

```cpp
std::visit(Overload{
    [](const ImuMeasurement& imu) { handle_imu(imu); },
    [](const GpsMeasurement& gps) { handle_gps(gps); }
}, measurement);
```

This is a compact way to write type-specific logic while keeping the set of cases explicit.
