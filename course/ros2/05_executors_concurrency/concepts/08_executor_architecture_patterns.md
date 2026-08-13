# 7. Executor Architecture Patterns

### Pattern A — Single node, single thread (most common)

```
main() → spin(node)
             │
    SingleThreadedExecutor
             │
     ┌───────┴───────┐
     │ sub_cb   timer│  serialised
     └───────────────┘
```

Use for: most sensor drivers, publishers, monitors.

### Pattern B — Single node, multi-thread with groups

```
main() → MultiThreadedExecutor(2)
               │
    ┌──────────┴──────────┐
    thread-0           thread-1
       │                  │
  [group_A_cb]       [group_B_cb]   ← can run simultaneously
```

Use for: high-frequency sensor ingestion + slow processing in same node.

### Pattern C — Multiple nodes, one executor

```cpp
auto exec = rclcpp::executors::SingleThreadedExecutor();
exec.add_node(driver_node);
exec.add_node(filter_node);
exec.spin();
```

Use for: composable node containers — both nodes share one thread, reducing
context-switch overhead versus separate processes.
