#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("config_echo_node");
  node->declare_parameter<std::string>("robot_name", "robot");
  node->declare_parameter<std::string>("config_profile", "dev");
  node->declare_parameter<double>("status_timeout_s", 1.0);

  auto publisher = node->create_publisher<std_msgs::msg::String>("config/summary", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [node, publisher]() {
      // TODO: Read parameters and publish a stable configuration summary.
      std_msgs::msg::String msg;
      msg.data = "TODO: robot/config summary";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
