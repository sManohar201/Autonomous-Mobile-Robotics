# 1. The Executor Model

### What an executor does

A ROS2 node does not run its own threads. It registers callbacks (subscriptions,
timers, services, action servers) and hands them to an **executor**. The executor
owns one or more threads and calls your callbacks on those threads.

```
rclcpp::Node  →  Executor  →  OS thread(s)  →  your callbacks
```

Without spinning the executor, **no callbacks fire**. `rclcpp::spin(node)` is
syntactic sugar for:

```cpp
auto exec = rclcpp::executors::SingleThreadedExecutor();
exec.add_node(node);
exec.spin();          // blocks here until shutdown
```

### The three built-in executors

| Executor | Threads | When to use |
|---|---|---|
| `SingleThreadedExecutor` | 1 | Default — no shared-state hazards, simplest mental model |
| `MultiThreadedExecutor(N)` | N | When you need concurrent callback processing |
| `StaticSingleThreadedExecutor` | 1 | Lower overhead — graph cannot change after spin starts |

```cpp
// Multi-threaded: 4 worker threads
rclcpp::executors::MultiThreadedExecutor exec{
    rclcpp::ExecutorOptions{}, 4};
exec.add_node(node);
exec.spin();
```

> **Production trade-off:** MultiThreadedExecutor only helps if the callback
> groups allow concurrency (see §3). Adding threads without configuring groups
> gives you race conditions for free without any throughput gain.
