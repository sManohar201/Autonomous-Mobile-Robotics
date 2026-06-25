#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("gazebo_monitor");
  auto last_clock = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);

  auto clock_sub = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "clock", 10,
    [last_clock](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
      // TODO: Track /clock startup and detect when simulation is advancing.
      *last_clock = msg->clock;
    });

  auto publisher = node->create_publisher<std_msgs::msg::String>("gazebo/monitor_status", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [publisher, last_clock]() {
      std_msgs::msg::String msg;
      msg.data = "TODO: handle Gazebo startup delays without false failures";
      publisher->publish(msg);
      (void)last_clock;
    });

  (void)clock_sub;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
