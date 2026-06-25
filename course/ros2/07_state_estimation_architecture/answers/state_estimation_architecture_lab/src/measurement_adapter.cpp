#include <cmath>
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
  const auto & q = msg.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  PoseMeasurement measurement;
  measurement.frame_id = msg.header.frame_id;
  measurement.x = msg.pose.pose.position.x;
  measurement.y = msg.pose.pose.position.y;
  measurement.yaw = std::atan2(siny_cosp, cosy_cosp);
  measurement.covariance_trace =
    msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[35];
  return measurement;
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
      line << "frame=" << measurement.frame_id
           << " x=" << measurement.x
           << " y=" << measurement.y
           << " yaw=" << measurement.yaw
           << " covariance_trace=" << measurement.covariance_trace;
      summary.data = line.str();
      output->publish(summary);
    });

  (void)sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
