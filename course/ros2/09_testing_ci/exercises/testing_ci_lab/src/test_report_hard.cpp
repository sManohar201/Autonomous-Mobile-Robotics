#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_report_hard");
  auto publisher = node->create_publisher<std_msgs::msg::String>("ci/test_matrix", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [publisher]() {
      // TODO: Publish a CI matrix that separates build, unit, launch, lint, and sim jobs.
      std_msgs::msg::String msg;
      msg.data = "TODO: CI test matrix";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
