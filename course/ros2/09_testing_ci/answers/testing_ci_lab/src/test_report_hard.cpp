#include <array>
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct CiJob
{
  const char * name;
  const char * trigger;
  const char * required_artifact;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_report_hard");
  constexpr std::array<CiJob, 5> jobs{{
    {"build", "every PR", "install log"},
    {"unit", "every PR", "gtest/junit"},
    {"launch", "every PR", "launch test log"},
    {"lint", "every PR", "ament lint report"},
    {"simulation", "nightly or gated", "bag replay summary"},
  }};

  auto publisher = node->create_publisher<std_msgs::msg::String>("ci/test_matrix", 10);
  auto timer = node->create_wall_timer(
    std::chrono::seconds(1),
    [publisher, jobs]() {
      std::ostringstream out;
      for (const auto & job : jobs) {
        out << job.name << "[trigger=" << job.trigger
            << ",artifact=" << job.required_artifact << "] ";
      }
      std_msgs::msg::String msg;
      msg.data = out.str();
      publisher->publish(msg);
    });

  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
