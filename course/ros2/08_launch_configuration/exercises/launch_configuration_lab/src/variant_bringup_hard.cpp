#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool is_supported_environment(const std::string & environment)
{
  // TODO: Accept only development, simulation, and field profiles.
  (void)environment;
  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("variant_bringup_selector");
  node->declare_parameter<std::string>("environment", "dev");
  node->declare_parameter<std::string>("robot_name", "robot");

  const auto environment = node->get_parameter("environment").as_string();
  if (!is_supported_environment(environment)) {
    RCLCPP_ERROR(node->get_logger(), "TODO: validate launch environment");
  }

  auto publisher = node->create_publisher<std_msgs::msg::String>("bringup/selected_profile", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [publisher]() {
      std_msgs::msg::String msg;
      msg.data = "TODO: selected launch/config profile";
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
