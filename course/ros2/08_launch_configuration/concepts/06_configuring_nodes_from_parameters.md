# 5. Configuring Nodes from Parameters

Nodes should read all tunable values from parameters, not from compile-time
constants or environment variables.

```cpp
#include <memory>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("config_echo_node");

    node->declare_parameter<std::string>("robot_name", "robot");
    node->declare_parameter<std::string>("config_profile", "dev");
    node->declare_parameter<double>("status_timeout_s", 1.0);

    auto pub = node->create_publisher<std_msgs::msg::String>("config/summary", 10);
    auto timer = node->create_wall_timer(std::chrono::seconds(1), [node, pub] {
        std::ostringstream out;
        out << "robot="     << node->get_parameter("robot_name").as_string()
            << " profile="  << node->get_parameter("config_profile").as_string()
            << " timeout_s=" << node->get_parameter("status_timeout_s").as_double()
            << " use_sim_time=" << (node->get_parameter("use_sim_time").as_bool()
                                   ? "true" : "false");
        std_msgs::msg::String msg;
        msg.data = out.str();
        pub->publish(msg);
    });

    (void)timer;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```

`use_sim_time` is a built-in parameter — always inject it via
`parameters=[{'use_sim_time': True}]` in the launch file, not in node code.
