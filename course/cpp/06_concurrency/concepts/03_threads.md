# 6.2 Threads

`std::thread` starts a new execution thread. A joinable thread must be joined or detached before destruction.

```cpp
std::thread worker{[] { do_work(); }};
worker.join();
```

Prefer joining. Detached threads are hard to reason about and dangerous during shutdown.
