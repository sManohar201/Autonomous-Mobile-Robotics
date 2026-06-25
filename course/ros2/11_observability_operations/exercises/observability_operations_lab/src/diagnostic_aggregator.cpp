#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("diagnostic_aggregator");
  auto latest = std::make_shared<std::string>("unknown");

  auto input = node->create_subscription<std_msgs::msg::String>(
    "subsystem/status", 10,
    [latest](const std_msgs::msg::String::SharedPtr msg) {
      // TODO: Aggregate all subsystem statuses into OK, DEGRADED, or FAILED.
      *latest = msg->data;
    });

  auto output = node->create_publisher<std_msgs::msg::String>("operator/health", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [output, latest]() {
      std_msgs::msg::String msg;
      msg.data = "TODO: aggregate health from " + *latest;
      output->publish(msg);
    });

  (void)input;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
