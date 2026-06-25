#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

bool is_stale(const rclcpp::Time & now, const rclcpp::Time & last_seen, double timeout_s)
{
  // TODO: Implement pure freshness logic so it can be unit-tested without ROS spinning.
  (void)now;
  (void)last_seen;
  (void)timeout_s;
  return false;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("freshness_logic_node");
  node->declare_parameter<double>("timeout_s", 1.0);
  auto last_seen = std::make_shared<rclcpp::Time>(0, 0, RCL_ROS_TIME);

  auto input = node->create_subscription<std_msgs::msg::String>(
    "test/input", 10,
    [node, last_seen](const std_msgs::msg::String::SharedPtr msg) {
      (void)msg;
      *last_seen = node->now();
    });

  auto output = node->create_publisher<std_msgs::msg::String>("test/freshness", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(200),
    [node, last_seen, output]() {
      std_msgs::msg::String msg;
      msg.data = is_stale(node->now(), *last_seen, node->get_parameter("timeout_s").as_double())
        ? "STALE" : "OK";
      output->publish(msg);
    });

  (void)input;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
