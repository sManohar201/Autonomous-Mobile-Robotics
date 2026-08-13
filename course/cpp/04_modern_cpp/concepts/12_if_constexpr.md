# 4.11 `if constexpr`

`if constexpr` was introduced in Module 3 for templates. In modern C++, it often replaces complicated overload sets for small compile-time decisions.

```cpp
template <typename Msg>
std::string frame_id_for() {
    if constexpr (std::is_same_v<Msg, ImuMeasurement>) {
        return "imu_link";
    } else if constexpr (std::is_same_v<Msg, GpsMeasurement>) {
        return "gps_link";
    } else {
        return "base_link";
    }
}
```

The discarded branches are not instantiated for the current type.
