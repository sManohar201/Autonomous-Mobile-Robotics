#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("package_boundary_review");
    const auto robot_name = node->declare_parameter<std::string>("robot_name", "robot1");
    auto pub = node->create_publisher<std_msgs::msg::String>("/architecture/package_report", 10);

    auto timer = node->create_wall_timer(1s, [pub, robot_name] {
        std_msgs::msg::String msg;
        msg.data = "robot=" + robot_name +
                   " interfaces=contracts monitoring=health bringup=orchestration description=model";
        pub->publish(msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

