# 7. Writing Nodes Portable Between Sim and Hardware

The test: can you run the exact same binary against `ros2 topic pub` and
against Gazebo without any code change?

**Yes if:** The node reads sensor topics, uses `node->now()` for time, and
has `use_sim_time` injected by the launch file.

**No if:** The node checks for a Gazebo-specific topic name, forks based on
`use_sim_time`, or calls Gazebo APIs directly.

```cpp
// PORTABLE — topic name and QoS come from parameters / launch file
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu_topic",  // parameterised in launch, not hardcoded
    rclcpp::QoS(10).best_effort(),
    callback);

// NOT PORTABLE — hardcoded Gazebo topic name
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "/world/default/model/robot/link/imu_link/sensor/imu/imu",  // DO NOT DO THIS
    ...);
```
