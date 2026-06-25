// Hard Exercise - Command and Telemetry API
//
// Design a node exposing both a telemetry stream and a command interface:
//   - Publish mission telemetry on /mission/telemetry at 1 Hz
//   - Expose a /mission/reset service that accepts commands and reports success
//
// Choose appropriate QoS for each: telemetry vs. commands have different
// reliability and durability requirements. Document your choices in comments.
//
// Think about: why is a service more appropriate than a topic for reset commands?

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
