// Hard Exercise - Lifecycle-Style Processor
//
// Build a node that models its own operational states (inactive, active, failed)
// and publishes health diagnostics based on input freshness.
//
// When active:
//   - subscribe to /processor/input
//   - publish "OK" on /processor/diagnostics while input arrives on time
//   - publish "DEGRADED stale input" when no message arrives within 1 second
// When not active: publish "INACTIVE"
//
// Use rclcpp::Time (node clock) for stale detection — not wall clock.
// Design your own state representation and transition logic.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
