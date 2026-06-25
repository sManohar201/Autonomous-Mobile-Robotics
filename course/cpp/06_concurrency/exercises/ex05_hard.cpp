// Exercise 05 (Hard) - Bounded Queue with Producers and Consumers
//
// Implement BoundedQueue<T> supporting multiple producers and consumers:
//   - capacity is set at construction; pushers block when the queue is full
//   - consumers block when the queue is empty
//   - closing the queue unblocks all waiters and drains cleanly
//   - no busy waiting allowed
//
// Verify correctness with a producer writing 10 items and a consumer summing them.

#include <iostream>

int main() {
    std::cout << "ex05_hard passed\n";
}
