# 4.9 `std::variant`

`std::variant` stores one value from a fixed set of alternatives.

```cpp
struct ImuMeasurement { double ax, ay, az; };
struct GpsMeasurement { double lat, lon, alt; };

using Measurement = std::variant<ImuMeasurement, GpsMeasurement>;
```

Visit a variant with `std::visit`:

```cpp
std::visit([](const auto& msg) {
    std::cout << "received measurement\n";
}, measurement);
```

Use variants when all alternatives are known and closed. Use runtime polymorphism when new alternatives must be added through plugins or dynamic loading.
