# 2. Simulation Time

### The `/clock` topic

Gazebo publishes `rosgraph_msgs/Clock` on `/clock`. Every node with
`use_sim_time: true` uses this as its time source instead of the OS clock.

```cpp
// With use_sim_time: true in launch file:
auto now = node->now();   // returns Gazebo simulation time
```

### Startup ordering problem

Nodes start before Gazebo emits any clock messages. A node with `use_sim_time`
will hang in `node->now()` until the first `/clock` message arrives.

```
t=0s: ros_gz_bridge starts → /clock not yet published
t=2s: your_node starts, calls node->now() → BLOCKS
t=3s: Gazebo ready, /clock starts → node unblocks
```

The CLAUDE.md `TimerAction(5s)` in `gazebo.launch.py` handles this: the bridge
starts immediately to establish `/clock`, and all other nodes start 5 s later.

### Checking the clock source at runtime

```cpp
auto timer = node->create_wall_timer(500ms, [node, pub] {
    const auto now = node->now();
    std::ostringstream s;
    s << "use_sim_time="
      << (node->get_parameter("use_sim_time").as_bool() ? "true" : "false")
      << " now_s=" << now.seconds()
      << " clock_type=" << now.get_clock_type();
    std_msgs::msg::String msg;
    msg.data = s.str();
    pub->publish(msg);
});
```

`clock_type` returns `3` for `RCL_ROS_TIME` (sim time) and `1` for
`RCL_SYSTEM_TIME` (wall time). A mismatch between publisher and subscriber
clock types causes timestamp arithmetic to fail.
