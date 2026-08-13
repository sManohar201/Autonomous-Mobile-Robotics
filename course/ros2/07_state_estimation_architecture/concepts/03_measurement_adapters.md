# 2. Measurement Adapters

A measurement adapter converts a ROS2 message into a plain C++ struct that the
filter understands. The adapter also extracts the information the filter needs
(e.g., yaw from quaternion).

### Why extract yaw from the quaternion?

`nav_msgs/Odometry` carries orientation as a quaternion `[x, y, z, w]`. A 2D
state estimator only needs yaw. Extracting it in the adapter keeps the filter
core free of quaternion math:

```cpp
struct PoseMeasurement {
    std::string frame_id;
    double x{};
    double y{};
    double yaw{};
    double covariance_trace{};
};

PoseMeasurement adapt_odometry(const nav_msgs::msg::Odometry& msg) {
    const auto& q = msg.pose.pose.orientation;

    // Yaw from quaternion: atan2(2(wz + xy), 1 - 2(yy + zz))
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

    PoseMeasurement m;
    m.frame_id = msg.header.frame_id;
    m.x = msg.pose.pose.position.x;
    m.y = msg.pose.pose.position.y;
    m.yaw = std::atan2(siny_cosp, cosy_cosp);
    // Trace of x, y, yaw covariance sub-block
    m.covariance_trace = msg.pose.covariance[0]   // var(x)
                       + msg.pose.covariance[7]   // var(y)
                       + msg.pose.covariance[35]; // var(yaw)
    return m;
}
```

The covariance trace (sum of diagonal elements) gives a single scalar for
"how uncertain is this measurement overall."

### Using the adapter

```cpp
auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [output](const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto m = adapt_odometry(*msg);

        std::ostringstream line;
        line << "frame=" << m.frame_id
             << " x=" << m.x << " y=" << m.y
             << " yaw=" << m.yaw
             << " cov_trace=" << m.covariance_trace;

        std_msgs::msg::String out;
        out.data = line.str();
        output->publish(out);
    });
```
