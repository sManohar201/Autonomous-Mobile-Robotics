# 5.9 Time-Based Windows

A time window stores recent samples and evicts old ones:

```cpp
while (!samples.empty() && now - samples.front().stamp > window) {
    samples.pop_front();
}
```

Use `std::deque` for efficient front removal and back insertion.
