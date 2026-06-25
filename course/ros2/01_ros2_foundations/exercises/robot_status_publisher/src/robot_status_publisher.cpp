// Moderate Exercise - Parameterized Robot Status Publisher
//
// Implement a node that:
//   1. Declares parameter rate_hz (default 1.0) — reject <= 0
//   2. Publishes std_msgs/String on /robot/status_text at that rate
//   3. Includes an incrementing counter in each message: "status count=N"
//   4. Exposes a std_srvs/Trigger service reset_counter that resets it to 0
//
// Hint: compute the timer period as 1.0/rate_hz seconds, cast to nanoseconds.
// Hint: the service callback receives request/response shared_ptrs.

#include <chrono>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // TODO: implement and spin RobotStatusPublisher node
    rclcpp::shutdown();
    return 0;
}
