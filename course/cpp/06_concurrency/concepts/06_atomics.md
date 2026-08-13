# 6.5 Atomics

Use `std::atomic<T>` for simple shared flags and counters.

```cpp
std::atomic<bool> stop{false};
```

Atomics do not make compound data structures safe. Use a mutex for multi-field invariants.
