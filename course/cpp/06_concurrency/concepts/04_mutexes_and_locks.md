# 6.3 Mutexes and Locks

Use `std::mutex` to protect shared data. Use RAII locks, not manual lock/unlock.

```cpp
std::mutex m;
std::vector<int> data;

{
    std::lock_guard<std::mutex> lock{m};
    data.push_back(1);
}
```

Keep critical sections short. Do not hold locks while calling unknown callbacks.
