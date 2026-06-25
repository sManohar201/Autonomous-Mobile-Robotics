#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sim_time_check");
  auto publisher = node->create_publisher<std_msgs::msg::String>("sim_time/status", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [node, publisher]() {
      // TODO: Publish enough state to compare wall time and sim time behavior.
      std_msgs::msg::String msg;
      msg.data = "TODO: sim time status";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
