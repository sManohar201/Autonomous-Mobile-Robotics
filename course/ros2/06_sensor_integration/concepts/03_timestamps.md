# 2. Timestamps

### What `header.stamp` means

Every stamped sensor message contains:
```
std_msgs/Header header
  builtin_interfaces/Time stamp   ← when the measurement was taken
  string frame_id                 ← coordinate frame
```

`stamp` is **measurement time**, not receive time. The driver sets it to the
hardware trigger time (or the best available estimate). It tells downstream
consumers where in time this sample belongs.

```cpp
auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10,
    [node](const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Age of sample at receive time
        auto age = node->now() - msg->header.stamp;
        RCLCPP_DEBUG(node->get_logger(), "IMU age: %.1f ms",
                     age.seconds() * 1e3);
    });
```

### Wall time vs sim time

| Mode | `node->now()` returns | When to use |
|---|---|---|
| Wall time (default) | OS real time | Real robot |
| Sim time (`use_sim_time: true`) | `/clock` topic time | Gazebo simulation |

With sim time enabled, `node->now()` blocks until the first `/clock` message
arrives. If the bridge is not running, the node hangs at startup.

```python
# launch file — enable sim time for simulation nodes
Node(
    package='my_pkg',
    executable='my_node',
    parameters=[{'use_sim_time': True}]
)
```

> **Production trade-off:** Never hardcode `rclcpp::Clock(RCL_SYSTEM_TIME)` in
> a node that may run in simulation. Always use `node->get_clock()` or
> `node->now()` — the correct clock source is injected by the runtime.

### Detecting stale data

A message is "stale" if its stamp is older than a threshold relative to now.
The threshold depends on the sensor rate:

```
IMU at 100 Hz → expect a sample every 10 ms → stale after 100 ms
LiDAR at 10 Hz → expect a sample every 100 ms → stale after 1 s
```

```cpp
bool is_stale(const rclcpp::Time& stamp, const rclcpp::Node::SharedPtr& node,
              double threshold_seconds) {
    return (node->now() - stamp).seconds() > threshold_seconds;
}
```

> **Caution:** `node->now() - msg->header.stamp` can throw if the two `Time`
> objects have different clock types (RCL_SYSTEM_TIME vs RCL_ROS_TIME). Use
> `rclcpp::Duration` with the node clock to stay consistent.
