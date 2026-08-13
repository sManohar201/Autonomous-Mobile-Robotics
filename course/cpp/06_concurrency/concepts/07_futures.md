# 6.6 Futures

`std::async` and `std::future` are useful for one-shot asynchronous results.

```cpp
auto result = std::async(std::launch::async, compute_map);
Map map = result.get();
```

For long-running pipelines, explicit threads and queues are usually clearer.
