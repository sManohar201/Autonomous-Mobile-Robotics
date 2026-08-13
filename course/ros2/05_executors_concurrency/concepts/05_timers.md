# 4. Timers

Timers fire a callback periodically. They are callbacks like any other — subject
to executor scheduling and callback group rules.

```cpp
using namespace std::chrono_literals;

// Wall timer: fires every 100ms of real time
auto timer = node->create_wall_timer(100ms, [this] {
    publish_diagnostics();
});

// Timer with a lambda capturing shared state
auto pub = node->create_publisher<std_msgs::msg::String>("/status", 10);
auto timer2 = node->create_wall_timer(1s, [pub] {
    std_msgs::msg::String msg;
    msg.data = "alive";
    pub->publish(msg);
});
```

### Timer accuracy

`create_wall_timer` uses real (wall) time, not sim time. For simulation:

```cpp
// Sim-time aware timer — only fires when /clock advances
auto timer = node->create_timer(
    node->get_clock(),
    rclcpp::Duration::from_seconds(0.1),
    [this] { check_health(); });
```

> **See also:** https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros.html
> for sim-time patterns with TF2.

### Timer jitter sources

1. **Callback queuing delay** — if a long subscription callback is running,
   the timer callback waits in the ready queue.
2. **OS scheduling** — the executor thread is preempted.
3. **DDS discovery storms** — brief CPU spikes on graph changes.

Keep timer callbacks short. If processing takes > timer period, consider
off-loading to a separate thread with a bounded queue.
