# 6.4 Condition Variables

`std::condition_variable` lets a thread sleep until data is available.

```cpp
cv.wait(lock, [&] { return ready; });
```

Always wait with a predicate. Condition variables can wake spuriously.
