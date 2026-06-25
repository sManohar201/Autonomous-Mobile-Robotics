#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

class StateEstimator
{
public:
  void predict(const rclcpp::Time & stamp)
  {
    // TODO: Advance the estimator state to the supplied stamp.
    (void)stamp;
  }

  void update(const nav_msgs::msg::Odometry & measurement)
  {
    // TODO: Reject stale/wrong-frame/high-covariance measurements before update.
    (void)measurement;
  }

  void reset()
  {
    // TODO: Reset filter state and diagnostics.
  }

  std::string diagnostics() const
  {
    return "TODO: publish estimator health";
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("localization_shell");
  auto estimator = std::make_shared<StateEstimator>();
  auto diagnostics = node->create_publisher<std_msgs::msg::String>("localization/status", 10);

  auto sub = node->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [estimator](const nav_msgs::msg::Odometry::SharedPtr msg) {
      estimator->predict(msg->header.stamp);
      estimator->update(*msg);
    });

  auto reset = node->create_service<std_srvs::srv::Trigger>(
    "localization/reset",
    [estimator](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      (void)request;
      estimator->reset();
      response->success = false;
      response->message = "TODO: implement reset contract";
    });

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),
    [estimator, diagnostics]() {
      std_msgs::msg::String msg;
      msg.data = estimator->diagnostics();
      diagnostics->publish(msg);
    });

  (void)sub;
  (void)reset;
  (void)timer;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
