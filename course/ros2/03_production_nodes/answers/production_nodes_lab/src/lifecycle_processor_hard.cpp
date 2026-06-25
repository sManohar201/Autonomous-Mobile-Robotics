#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

enum class ProcessorState { Inactive, Active, Failed };

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lifecycle_processor_hard");
    auto state = std::make_shared<ProcessorState>(ProcessorState::Active);
    auto last_seen = std::make_shared<rclcpp::Time>(node->now());
    const auto timeout = rclcpp::Duration::from_seconds(1.0);
    auto pub = node->create_publisher<std_msgs::msg::String>("/processor/diagnostics", 10);
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "/processor/input", 10,
        [last_seen, node](const std_msgs::msg::String&) { *last_seen = node->now(); });
    auto timer = node->create_wall_timer(200ms, [node, pub, state, last_seen, timeout] {
        std_msgs::msg::String msg;
        if (*state != ProcessorState::Active) msg.data = "INACTIVE";
        else if ((node->now() - *last_seen) > timeout) msg.data = "DEGRADED stale input";
        else msg.data = "OK";
        pub->publish(msg);
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

