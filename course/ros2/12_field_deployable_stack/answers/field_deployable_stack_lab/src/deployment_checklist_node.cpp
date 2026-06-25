#include <memory>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("deployment_checklist_node");
  node->declare_parameter<bool>("config_loaded", false);
  node->declare_parameter<bool>("diagnostics_ready", false);
  node->declare_parameter<bool>("bagging_ready", false);
  node->declare_parameter<bool>("operator_confirmed", false);

  auto publisher = node->create_publisher<std_msgs::msg::String>("deployment/readiness", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [node, publisher]() {
      const bool config_loaded = node->get_parameter("config_loaded").as_bool();
      const bool diagnostics_ready = node->get_parameter("diagnostics_ready").as_bool();
      const bool bagging_ready = node->get_parameter("bagging_ready").as_bool();
      const bool operator_confirmed = node->get_parameter("operator_confirmed").as_bool();
      const bool ready = config_loaded && diagnostics_ready && bagging_ready && operator_confirmed;

      std::ostringstream out;
      out << "ready=" << (ready ? "true" : "false")
          << " config_loaded=" << config_loaded
          << " diagnostics_ready=" << diagnostics_ready
          << " bagging_ready=" << bagging_ready
          << " operator_confirmed=" << operator_confirmed;
      std_msgs::msg::String msg;
      msg.data = out.str();
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
