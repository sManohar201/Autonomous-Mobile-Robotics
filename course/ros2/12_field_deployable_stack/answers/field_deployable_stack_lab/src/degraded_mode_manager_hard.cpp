#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::string select_mode(const std::string & subsystem_health)
{
  if (subsystem_health.find("control:FAILED") != std::string::npos ||
      subsystem_health.find("safety:FAILED") != std::string::npos) {
    return "STOP";
  }
  if (subsystem_health.find("localization:FAILED") != std::string::npos) {
    return "LOCALIZE_ONLY";
  }
  if (subsystem_health.find("sensor:DEGRADED") != std::string::npos ||
      subsystem_health.find("planning:DEGRADED") != std::string::npos) {
    return "SLOW";
  }
  return "NORMAL";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("degraded_mode_manager");
  auto health = std::make_shared<std::string>("unknown");

  auto sub = node->create_subscription<std_msgs::msg::String>(
    "subsystem/health", 10,
    [health](const std_msgs::msg::String::SharedPtr msg) {
      *health = msg->data;
    });

  auto output = node->create_publisher<std_msgs::msg::String>("robot/degraded_mode", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [output, health]() {
      std_msgs::msg::String msg;
      msg.data = select_mode(*health);
      output->publish(msg);
    });

  (void)sub;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
