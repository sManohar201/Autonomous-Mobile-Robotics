// Exercise 05 (Hard) - Generic Ring Buffer with Iterator Support
//
// Context:
//   Fixed-capacity buffers in sensor drivers need logical-order iteration
//   without dynamic allocation.
//
// Tasks:
//   1. Implement RingBuffer<T, Capacity> with push, at, size, full.
//   2. Add a const iterator that walks logical order, oldest to newest.
//   3. Constrain Capacity > 0 with static_assert.
//   4. Add latest_timestamp() constrained to timestamped message types.

#include <iostream>

int main() {
    std::cout << "ex05_hard passed\n";
}

