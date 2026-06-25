#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("snapshot_processor");
    auto latest = std::make_shared<std::optional<std::string>>();
    auto mutex = std::make_shared<std::mutex>();
    auto pub = node->create_publisher<std_msgs::msg::String>("/processed/status", 10);
    auto sub = node->create_subscription<std_msgs::msg::String>("/input/status", 10, [latest, mutex](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(*mutex); *latest = msg->data;
    });
    auto timer = node->create_wall_timer(100ms, [latest, mutex, pub] {
        std::optional<std::string> snapshot;
        { std::lock_guard<std::mutex> lock(*mutex); snapshot = *latest; }
        std_msgs::msg::String out; out.data = snapshot ? "processed " + *snapshot : "missing input"; pub->publish(out);
    });
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
