#include <memory>
#include <sstream>

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
      const auto now = node->now();
      std::ostringstream out;
      out << "use_sim_time=" << (node->get_parameter("use_sim_time").as_bool() ? "true" : "false")
          << " now_s=" << now.seconds()
          << " clock_type=" << now.get_clock_type();
      std_msgs::msg::String msg;
      msg.data = out.str();
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
