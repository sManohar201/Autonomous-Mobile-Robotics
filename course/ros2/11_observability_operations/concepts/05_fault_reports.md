# 4. Fault Reports

A fault report is a structured message that tells an operator:
- **What** failed (subsystem name)
- **How bad** it is (severity)
- **Why** it failed (reason)
- **What to do** (action)

```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("fault_report_hard");

    node->declare_parameter<std::string>("subsystem", "localization");
    node->declare_parameter<std::string>("severity",  "DEGRADED");
    node->declare_parameter<std::string>("reason",    "stale odometry");
    node->declare_parameter<std::string>("action",    "verify odom publisher and inspect latest bag");

    auto pub = node->create_publisher<std_msgs::msg::String>(
        "operator/fault_report", 10);

    auto timer = node->create_wall_timer(std::chrono::seconds(1),
        [node, pub] {
            std::ostringstream s;
            s << "stamp_ns=" << node->now().nanoseconds()
              << " subsystem=" << node->get_parameter("subsystem").as_string()
              << " severity="  << node->get_parameter("severity").as_string()
              << " reason=\""  << node->get_parameter("reason").as_string()  << "\""
              << " action=\""  << node->get_parameter("action").as_string()  << "\"";
            std_msgs::msg::String msg;
            msg.data = s.str();
            pub->publish(msg);
        });

    (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Making these values parameters means the same binary can report any fault — the
launch file or operator inject the specifics at runtime.
