# 5.2 Containers

Use `std::vector` for contiguous dynamic arrays and numeric data. Use `std::deque` when pushing and popping at both ends. Use `std::array` for fixed-size stack storage. Use `std::map` or `std::unordered_map` for key/value lookup.

```cpp
std::vector<double> ranges;
std::array<double, 3> accel{};
std::deque<ImuSample> imu_window;
std::unordered_map<std::string, double> rates;
```

Container choice is an engineering decision about access pattern, allocation, ordering, and invalidation.
