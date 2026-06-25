#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("deployment_checklist_node");
  node->declare_parameter<bool>("config_loaded", false);
  node->declare_parameter<bool>("diagnostics_ready", false);
  node->declare_parameter<bool>("bagging_ready", false);

  auto publisher = node->create_publisher<std_msgs::msg::String>("deployment/readiness", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [publisher]() {
      // TODO: Combine checklist parameters into a deployment readiness verdict.
      std_msgs::msg::String msg;
      msg.data = "TODO: deployment readiness";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
