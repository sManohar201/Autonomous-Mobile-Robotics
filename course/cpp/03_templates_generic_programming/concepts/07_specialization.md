# 3.6 Specialization

Specialization lets you customize a template for a specific type.

```cpp
template <typename T>
struct SensorTraits {
    static constexpr const char* topic = "/unknown";
    static constexpr double default_rate_hz = 10.0;
};

struct ImuSample {};
struct LidarScan {};

template <>
struct SensorTraits<ImuSample> {
    static constexpr const char* topic = "/imu/data";
    static constexpr double default_rate_hz = 100.0;
};

template <>
struct SensorTraits<LidarScan> {
    static constexpr const char* topic = "/front_laser/scan";
    static constexpr double default_rate_hz = 20.0;
};
```

Traits move type-specific facts out of generic algorithms:

```cpp
template <typename Msg>
void print_default_topic() {
    std::cout << SensorTraits<Msg>::topic << "\n";
}
```

This pattern appears in serialization libraries, ROS message adapters, Eigen traits, and numeric type systems.
