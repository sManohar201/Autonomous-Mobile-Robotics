// Moderate Exercise - QoS Status Publisher
//
// Publish /robot/status (std_msgs/String) with transient-local QoS
// so that late-joining subscribers receive the last published message immediately.
//
// Steps:
//   1. Create a QoS profile: depth=1, reliable, transient_local
//   2. Publish "OK" on /robot/status at 1 Hz
//
// Hint: rclcpp::QoS(1).reliable().transient_local() builds the profile.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
