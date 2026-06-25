#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("command_api_hard");
    auto telemetry_pub = node->create_publisher<std_msgs::msg::String>("/mission/telemetry", rclcpp::QoS(10).reliable());
    auto reset_srv = node->create_service<std_srvs::srv::Trigger>(
        "/mission/reset",
        [](
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            (void)request;
            response->success = true;
            response->message = "mission reset accepted";
        });
    auto timer = node->create_wall_timer(1s, [telemetry_pub] { std_msgs::msg::String msg; msg.data = "mission_state=IDLE"; telemetry_pub->publish(msg); });
    (void)reset_srv;
    rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
