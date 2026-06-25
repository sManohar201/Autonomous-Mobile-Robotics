#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("qos_status_publisher");
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto pub = node->create_publisher<std_msgs::msg::String>("/robot/status", qos);
    auto timer = node->create_wall_timer(1s, [pub] { std_msgs::msg::String msg; msg.data = "OK"; pub->publish(msg); });
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}

