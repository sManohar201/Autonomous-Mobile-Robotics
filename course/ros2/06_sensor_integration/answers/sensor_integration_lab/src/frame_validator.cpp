#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("frame_validator");
    const auto expected = node->declare_parameter<std::string>("expected_frame_id", "imu_link");
    auto pub = node->create_publisher<std_msgs::msg::String>("/sensor/frame_status", 10);
    auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, [pub, expected](const sensor_msgs::msg::Imu::SharedPtr msg) {
        std_msgs::msg::String out; out.data = msg->header.frame_id == expected ? "OK" : "FRAME_ERROR expected=" + expected + " actual=" + msg->header.frame_id; pub->publish(out);
    });
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
