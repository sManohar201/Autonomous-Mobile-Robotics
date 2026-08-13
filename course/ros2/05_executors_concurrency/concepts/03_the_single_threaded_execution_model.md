# 2. The Single-Threaded Execution Model

With `SingleThreadedExecutor` (the default), the executor processes one
callback at a time in round-robin order. This has a critical implication:

```
Thread 0:  [subscription_cb]  [timer_cb]  [service_cb]  [timer_cb]  ...
               ↑ one at a time, no overlap
```

**Consequence:** If your subscription callback takes 50 ms and your timer fires
every 20 ms, the timer will be late. Single-threaded execution serialises all
work.

```cpp
// Checking elapsed time in a timer — single-threaded is fine here
auto timer = node->create_wall_timer(20ms, [this] {
    // This runs every 20ms *if* other callbacks finish in time
    publish_status();
});
```

### When single-threaded is enough

- All callbacks are short (< 1 ms typical, < 5 ms worst-case)
- No blocking calls (no sleep, no file I/O, no slow network calls)
- Sensor fusion at 50–100 Hz: callbacks reading from the DDS queue are fast

Most production ROS2 nodes use `SingleThreadedExecutor` and simply keep
callbacks short.
