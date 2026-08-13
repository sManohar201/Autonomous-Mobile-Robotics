# 5. QoS for Sensor Topics

Different sensors have different reliability requirements:

| Sensor stream | Reliability | Durability | Depth | Why |
|---|---|---|---|---|
| IMU (100 Hz) | `best_effort` | `volatile` | 5–10 | High rate; old samples are worthless |
| Odometry (50 Hz) | `best_effort` | `volatile` | 5 | Same: latency matters more than delivery |
| LiDAR scan (10 Hz) | `reliable` | `volatile` | 5 | Lower rate; reliability worthwhile |
| GPS fix (1–10 Hz) | `reliable` | `transient_local` | 1 | Late-joining nodes need last known fix |
| Camera image (30 Hz) | `best_effort` | `volatile` | 1 | Bandwidth-dominated; drop is acceptable |

```cpp
// Best-effort subscriber for high-rate IMU
auto qos = rclcpp::QoS(10).best_effort();
auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", qos, callback);

// Reliable subscriber for GPS
auto gps_qos = rclcpp::QoS(1).reliable().transient_local();
auto gps_sub = node->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/gps/fix", gps_qos, callback);
```

> **Production trade-off:** If a subscriber uses `reliable` but the publisher
> uses `best_effort`, DDS won't establish the connection — the subscription
> silently receives nothing. Match QoS or explicitly use `keep_last(1)` with
> compatible profiles. Use `ros2 topic info -v /topic_name` to inspect QoS on
> both ends.
