// Hard Exercise - Robot Monitor Prototype
//
// Build a node that watches a status topic for freshness and publishes a summary.
//
// Parameters: input_topic ("/robot/status_text"), stale_timeout_ms (1000), check_period_ms (200)
// Validate: reject empty topic and non-positive timeout/period values.
//
// Subscribe to input_topic. On each message, record the last-received text and time.
// Every check_period_ms, publish /robot/monitor_summary (std_msgs/String):
//   "STALE no messages received"  — if no message ever arrived
//   "STALE last='<text>'"         — if last message is older than stale_timeout_ms
//   "OK last='<text>'"            — otherwise
//
// Use rclcpp::Time (node->now()) for time comparison — not std::chrono wall time.

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO: implement and spin RobotMonitorPrototype node
    rclcpp::shutdown();
    return 0;
}
