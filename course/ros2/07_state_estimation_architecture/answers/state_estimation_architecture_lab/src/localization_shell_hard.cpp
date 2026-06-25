#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class StateEstimator
{
public:
  explicit StateEstimator(std::string required_frame)
  : required_frame_(std::move(required_frame))
  {
  }

  void predict(const rclcpp::Time & stamp)
  {
    last_predict_stamp_ = stamp;
  }

  bool update(const nav_msgs::msg::Odometry & measurement, std::string & reason)
  {
    if (measurement.header.frame_id != required_frame_) {
      reason = "wrong_frame";
      ++rejected_;
      return false;
    }
    const double covariance_trace =
      measurement.pose.covariance[0] + measurement.pose.covariance[7] + measurement.pose.covariance[35];
    if (covariance_trace > max_covariance_trace_) {
      reason = "high_covariance";
      ++rejected_;
      return false;
    }
    const double dx = measurement.pose.pose.position.x - x_;
    const double dy = measurement.pose.pose.position.y - y_;
    if (initialized_ && std::hypot(dx, dy) > max_jump_m_) {
      reason = "outlier_jump";
      ++rejected_;
      return false;
    }

    x_ = measurement.pose.pose.position.x;
    y_ = measurement.pose.pose.position.y;
    last_update_stamp_ = measurement.header.stamp;
    initialized_ = true;
    ++accepted_;
    reason = "accepted";
    return true;
  }

  void reset()
  {
    x_ = 0.0;
    y_ = 0.0;
    accepted_ = 0;
    rejected_ = 0;
    initialized_ = false;
  }

  std::string diagnostics(const rclcpp::Clock & clock) const
  {
    std::ostringstream out;
    out << "initialized=" << (initialized_ ? "true" : "false")
        << " accepted=" << accepted_
        << " rejected=" << rejected_
        << " state=(" << x_ << "," << y_ << ")"
        << " predict_age_s=" << (clock.now() - last_predict_stamp_).seconds()
        << " update_age_s=" << (clock.now() - last_update_stamp_).seconds();
    return out.str();
  }

private:
  std::string required_frame_;
  double max_covariance_trace_{1.0};
  double max_jump_m_{3.0};
  double x_{0.0};
  double y_{0.0};
  int accepted_{0};
  int rejected_{0};
  bool initialized_{false};
  rclcpp::Time last_predict_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("localization_shell");
  node->declare_parameter<std::string>("required_frame", "odom");

  auto estimator = std::make_shared<StateEstimator>(
    node->get_parameter("required_frame").as_string());
  auto diagnostics = node->create_publisher<std_msgs::msg::String>("localization/status", 10);

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [estimator, node](const nav_msgs::msg::Odometry::SharedPtr msg) {
      estimator->predict(msg->header.stamp);
      std::string reason;
      const bool accepted = estimator->update(*msg, reason);
      if (!accepted) {
        RCLCPP_WARN_THROTTLE(
          node->get_logger(), *node->get_clock(), 2000,
          "Rejected localization measurement: %s", reason.c_str());
      }
    });

  auto reset = node->create_service<std_srvs::srv::Trigger>(
    "localization/reset",
    [estimator](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      (void)request;
      estimator->reset();
      response->success = true;
      response->message = "localization estimator reset";
    });

  auto timer = node->create_wall_timer(
    500ms,
    [estimator, diagnostics, node]() {
      std_msgs::msg::String msg;
      msg.data = estimator->diagnostics(*node->get_clock());
      diagnostics->publish(msg);
    });

  (void)sub;
  (void)reset;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
