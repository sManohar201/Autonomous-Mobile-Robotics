#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fault_report_hard");
  auto publisher = node->create_publisher<std_msgs::msg::String>("operator/fault_report", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [publisher]() {
      // TODO: Publish timestamp, subsystem, severity, reason, and suggested action.
      std_msgs::msg::String msg;
      msg.data = "TODO: structured fault report";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
