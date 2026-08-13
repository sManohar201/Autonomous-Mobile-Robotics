# 3. Node Tests — Single Node with Synthetic Topics

Node tests spin a real node and inject synthetic input via `ros2 topic pub` or
a test publisher. They verify that the node's topic graph behaves correctly.

### The freshness_logic_node

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool is_stale(const rclcpp::Time& now, const rclcpp::Time& last_seen,
              double timeout_s) {
    return last_seen.nanoseconds() == 0 ||
           (now - last_seen).seconds() > timeout_s;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("freshness_logic_node");
    node->declare_parameter<double>("timeout_s", 1.0);

    auto last_seen = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);

    auto input = node->create_subscription<std_msgs::msg::String>(
        "test/input", 10,
        [node, last_seen](const std_msgs::msg::String::SharedPtr) {
            *last_seen = node->now();
        });

    auto output = node->create_publisher<std_msgs::msg::String>(
        "test/freshness", 10);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(200),
        [node, last_seen, output] {
            const double timeout = node->get_parameter("timeout_s").as_double();
            std::ostringstream s;
            s << "state="
              << (is_stale(node->now(), *last_seen, timeout) ? "STALE" : "OK")
              << " timeout_s=" << timeout
              << " last_seen_ns=" << last_seen->nanoseconds();
            std_msgs::msg::String msg;
            msg.data = s.str();
            output->publish(msg);
        });

    (void)input; (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

### Testing this node manually

```bash
# Terminal 1: run the node
ros2 run testing_ci_lab freshness_logic_node

# Terminal 2: inject input
ros2 topic pub /test/input std_msgs/msg/String '{}' --rate 2

# Terminal 3: watch output
ros2 topic echo /test/freshness

# Expected: state=OK while /test/input is publishing
# Expected: state=STALE after > 1 s of silence
```
