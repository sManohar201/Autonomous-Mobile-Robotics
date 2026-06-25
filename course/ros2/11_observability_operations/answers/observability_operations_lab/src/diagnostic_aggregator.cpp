#include <map>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int severity_rank(const std::string & status)
{
  if (status.find("FAILED") != std::string::npos) {
    return 2;
  }
  if (status.find("DEGRADED") != std::string::npos || status.find("STALE") != std::string::npos) {
    return 1;
  }
  return 0;
}

const char * label_for(int rank)
{
  if (rank >= 2) {
    return "FAILED";
  }
  if (rank == 1) {
    return "DEGRADED";
  }
  return "OK";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("diagnostic_aggregator");
  auto statuses = std::make_shared<std::map<std::string, std::string>>();

  auto input = node->create_subscription<std_msgs::msg::String>(
    "subsystem/status", 10,
    [statuses](const std_msgs::msg::String::SharedPtr msg) {
      const auto split = msg->data.find(':');
      const auto name = split == std::string::npos ? std::string("unknown") : msg->data.substr(0, split);
      (*statuses)[name] = msg->data;
    });

  auto output = node->create_publisher<std_msgs::msg::String>("operator/health", 10);
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [output, statuses]() {
      int worst = 0;
      std::ostringstream out;
      out << "subsystems=" << statuses->size() << " ";
      for (const auto & [name, status] : *statuses) {
        worst = std::max(worst, severity_rank(status));
        out << name << "={" << status << "} ";
      }
      std_msgs::msg::String msg;
      msg.data = std::string("robot_health=") + label_for(worst) + " " + out.str();
      output->publish(msg);
    });

  (void)input;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
