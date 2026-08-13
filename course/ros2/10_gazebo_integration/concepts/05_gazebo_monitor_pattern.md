# 4. Gazebo Monitor Pattern

A monitoring node that watches the Gazebo clock: if the clock goes silent for
more than a grace period, report `WAITING_FOR_GAZEBO_CLOCK`:

```cpp
#include "rosgraph_msgs/msg/clock.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("gazebo_monitor");
    node->declare_parameter<double>("startup_grace_s", 5.0);

    // Wall-time baseline — used to compute age without blocking on sim clock
    auto first_wall   = std::make_shared<rclcpp::Time>(node->get_clock()->now());
    auto last_clock   = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);
    auto clock_count  = std::make_shared<int>(0);

    // Subscribe to /clock directly — not via node->now()
    auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "clock", 10,
        [last_clock, clock_count](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            *last_clock = msg->clock;
            ++(*clock_count);
        });

    auto pub = node->create_publisher<std_msgs::msg::String>("gazebo/monitor_status", 10);

    auto timer = node->create_wall_timer(500ms,
        [node, pub, first_wall, last_clock, clock_count] {
            const double grace_s   = node->get_parameter("startup_grace_s").as_double();
            const double wall_age  = (node->get_clock()->now() - *first_wall).seconds();
            const bool in_grace    = wall_age < grace_s;
            const bool clock_seen  = *clock_count > 0;

            std::ostringstream s;
            s << "state=" << ((clock_seen || in_grace) ? "OK" : "WAITING_FOR_GAZEBO_CLOCK")
              << " clock_messages=" << *clock_count
              << " last_clock_s=" << last_clock->seconds()
              << " grace_s=" << grace_s;
            std_msgs::msg::String msg;
            msg.data = s.str();
            pub->publish(msg);
        });

    (void)clock_sub; (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

Note: The timer uses `node->get_clock()->now()` not `node->now()`. When
`use_sim_time` is false (common for monitoring nodes that need wall time
regardless), the two are equivalent. When `use_sim_time` is true, `node->now()`
returns sim time — which may be zero before the clock starts.
