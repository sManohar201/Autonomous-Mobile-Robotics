# 5.6 `std::chrono`

Use `std::chrono` instead of raw integers for time.

```cpp
using Clock = std::chrono::steady_clock;
auto start = Clock::now();
std::chrono::milliseconds timeout{100};
```

`steady_clock` is monotonic and appropriate for measuring durations. System time can jump and is not appropriate for elapsed-time logic.
