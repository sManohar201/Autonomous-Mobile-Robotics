# 4.3 Structured Bindings

Structured bindings unpack tuple-like or aggregate values.

```cpp
struct Pose2d {
    double x;
    double y;
    double theta;
};

Pose2d pose{1.0, 2.0, 0.5};
auto [x, y, theta] = pose;
```

They are especially useful with maps:

```cpp
for (const auto& [name, rate_hz] : sensor_rates) {
    std::cout << name << " runs at " << rate_hz << " Hz\n";
}
```

Be intentional about copies:

```cpp
auto [k, v] = *it;        // copies key and value
auto& [rk, rv] = *it;     // references pair members
const auto& [ck, cv] = *it;
```
