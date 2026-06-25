#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("parameter_validation_node");
    const auto input_topic = node->declare_parameter<std::string>("input_topic", "/input");
    const auto output_topic = node->declare_parameter<std::string>("output_topic", "/status");
    const double expected_rate_hz = node->declare_parameter<double>("expected_rate_hz", 10.0);
    const int stale_timeout_ms = node->declare_parameter<int>("stale_timeout_ms", 500);
    if (input_topic.empty() || output_topic.empty()) throw std::runtime_error("topics must be non-empty");
    if (!std::isfinite(expected_rate_hz) || expected_rate_hz <= 0.0) throw std::runtime_error("expected_rate_hz invalid");
    if (stale_timeout_ms <= 0) throw std::runtime_error("stale_timeout_ms invalid");

    auto pub = node->create_publisher<std_msgs::msg::String>(output_topic, 10);
    auto timer = node->create_wall_timer(1s, [pub, input_topic] {
        std_msgs::msg::String msg;
        msg.data = "configured input=" + input_topic;
        pub->publish(msg);
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

