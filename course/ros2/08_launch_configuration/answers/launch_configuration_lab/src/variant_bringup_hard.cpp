#include <array>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool is_supported_environment(const std::string & environment)
{
  constexpr std::array<const char *, 3> allowed{"dev", "sim", "field"};
  for (const auto * item : allowed) {
    if (environment == item) {
      return true;
    }
  }
  return false;
}

std::string lifecycle_policy_for(const std::string & environment)
{
  if (environment == "field") {
    return "configure:drivers,monitoring,localization; activate:drivers,monitoring,localization";
  }
  if (environment == "sim") {
    return "configure:gazebo_bridge,monitoring,localization; activate:gazebo_bridge,monitoring,localization";
  }
  return "configure:monitoring; activate:monitoring";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("variant_bringup_selector");
  node->declare_parameter<std::string>("environment", "dev");
  node->declare_parameter<std::string>("robot_name", "robot");

  const auto environment = node->get_parameter("environment").as_string();
  if (!is_supported_environment(environment)) {
    RCLCPP_FATAL(node->get_logger(), "Unsupported environment: %s", environment.c_str());
    rclcpp::shutdown();
    return 2;
  }

  auto publisher = node->create_publisher<std_msgs::msg::String>("bringup/selected_profile", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [node, publisher, environment]() {
      std::ostringstream out;
      out << "robot=" << node->get_parameter("robot_name").as_string()
          << " environment=" << environment
          << " lifecycle_policy=" << lifecycle_policy_for(environment);
      std_msgs::msg::String msg;
      msg.data = out.str();
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
