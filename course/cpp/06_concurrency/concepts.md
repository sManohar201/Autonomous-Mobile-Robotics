# Module 6 - Concurrency

**Prerequisites:** Modules 1-5
**Goal:** Write safe concurrent C++ for robotics pipelines: producers, consumers, shared state, cancellation, and bounded queues.

---

## 6.1 Why Concurrency Matters

Robots run many activities at once: sensor acquisition, filtering, mapping, planning, logging, diagnostics, and control. C++ gives low-level concurrency primitives, but correctness depends on disciplined ownership and synchronization.

---

## 6.2 Threads

`std::thread` starts a new execution thread. A joinable thread must be joined or detached before destruction.

```cpp
std::thread worker{[] { do_work(); }};
worker.join();
```

Prefer joining. Detached threads are hard to reason about and dangerous during shutdown.

---

## 6.3 Mutexes and Locks

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

---

## 6.4 Condition Variables

`std::condition_variable` lets a thread sleep until data is available.

```cpp
cv.wait(lock, [&] { return ready; });
```

Always wait with a predicate. Condition variables can wake spuriously.

---

## 6.5 Atomics

Use `std::atomic<T>` for simple shared flags and counters.

```cpp
std::atomic<bool> stop{false};
```

Atomics do not make compound data structures safe. Use a mutex for multi-field invariants.

---

## 6.6 Futures

`std::async` and `std::future` are useful for one-shot asynchronous results.

```cpp
auto result = std::async(std::launch::async, compute_map);
Map map = result.get();
```

For long-running pipelines, explicit threads and queues are usually clearer.

---

## 6.7 Producer/Consumer Queues

A common robotics pattern is one thread producing sensor messages and another consuming them.

The queue needs:

- A mutex to protect storage
- A condition variable to block consumers
- A shutdown flag
- A capacity policy for overload

For real-time systems, prefer bounded queues and explicit drop behavior.

---

## 6.8 Practical Rules

- Do not share mutable data without synchronization.
- Use RAII locks.
- Wait on condition variables with predicates.
- Use atomics for simple flags only.
- Define shutdown behavior early.
- Avoid holding locks while executing callbacks.

