#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

struct PoseMeasurement
{
  std::string frame_id;
  double x{};
  double y{};
  double yaw{};
  double covariance_trace{};
};

PoseMeasurement adapt_odometry(const nav_msgs::msg::Odometry & msg)
{
  // TODO: Convert the ROS odometry message into the internal measurement type.
  (void)msg;
  return {};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("measurement_adapter");
  auto output = node->create_publisher<std_msgs::msg::String>("pose_measurement", 10);

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [output](const nav_msgs::msg::Odometry::SharedPtr msg) {
      const auto measurement = adapt_odometry(*msg);
      std_msgs::msg::String summary;
      std::ostringstream line;
      line << "TODO frame=" << measurement.frame_id
           << " x=" << measurement.x
           << " y=" << measurement.y
           << " covariance_trace=" << measurement.covariance_trace;
      summary.data = line.str();
      output->publish(summary);
    });

  (void)sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
