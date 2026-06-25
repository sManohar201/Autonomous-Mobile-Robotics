// Moderate Exercise - Parameter Validation Node
//
// Implement a node that validates its own configuration at startup.
//
// Steps:
//   1. Declare: input_topic (string), output_topic (string),
//               expected_rate_hz (double), stale_timeout_ms (int)
//   2. Reject invalid values: empty topics, non-positive/non-finite rate,
//      non-positive timeout — throw std::runtime_error with a clear message
//   3. Publish a startup status string on output_topic at 1 Hz
//
// Hint: std::isfinite() from <cmath> checks for inf/nan.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
