// Moderate Exercise - Package Boundary Review
//
// Publish a structured text report describing package roles in a ROS2 system.
//
// Steps:
//   1. Declare parameter robot_name (default "robot1")
//   2. Create a publisher on /architecture/package_report (std_msgs/String)
//   3. Publish at 1 Hz: "robot=<name> interfaces=... monitoring=... bringup=... description=..."
//
// Hint: create_wall_timer(1s, lambda) is the idiomatic way to publish periodically.
// Hint: lambdas can capture shared_ptr publishers and parameter values by value.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
