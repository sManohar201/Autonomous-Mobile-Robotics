// Exercise 03 (Hard) - Detection Idiom and Concepts for Sensor Messages
//
// Context:
//   Generic loggers need to accept any message with timestamp/frame fields,
//   while producing clear compiler errors for invalid message types.
//
// Tasks:
//   1. Define concepts HasTimestamp, HasFrameId, and TimedFrameMessage.
//   2. Implement latency_ns(now, msg), constrained to HasTimestamp.
//   3. Implement describe(msg), using if constexpr for optional frame_id.
//   4. Add static_assert tests for valid and invalid message structs.

#include <iostream>

int main() {
    std::cout << "ex03_hard passed\n";
}

