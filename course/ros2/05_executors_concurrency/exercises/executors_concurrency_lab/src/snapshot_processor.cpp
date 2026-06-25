// Moderate Exercise - Snapshot Processor
//
// Process incoming messages without holding a lock during processing.
//
// Steps:
//   1. Subscribe to /input/status; store the latest message under a mutex
//   2. Create a 100ms timer that:
//      a. Copies the latest message under lock (snapshot)
//      b. Releases the lock immediately
//      c. Publishes "processed <snapshot>" on /processed/status outside the lock
//   3. Publish "missing input" if no message has arrived yet
//
// Hint: use shared_ptr<optional<string>> + shared_ptr<mutex> to share state
// with the timer lambda without a class.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO
    rclcpp::shutdown();
    return 0;
}
