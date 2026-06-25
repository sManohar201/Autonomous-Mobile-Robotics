// Hard Exercise - Concurrent Sensor Pipeline
//
// Aggregate status from three independent sensor topics into a single summary.
//
// Subscribe to /scan_status, /imu_status, /odom_status (all std_msgs/String).
// Every 200ms publish /pipeline/summary:
//   "OK"              — if all three have received at least one message
//   "MISSING_INPUTS"  — otherwise
//
// Locking rule: hold the mutex only long enough to copy cached values.
// Do not hold the lock while publishing.
//
// Use callback groups if you want the subscriptions to run concurrently.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
