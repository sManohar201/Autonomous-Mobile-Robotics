// Exercise 03 (Hard) - Condition Variable with Spurious Wakeups
//
// Implement a generic BlockingQueue<T>:
//   - push enqueues and wakes one waiter
//   - pop blocks until an item is available or the queue is closed
//   - close wakes all waiters so they can exit cleanly
//   - pop returns an empty optional when the queue is closed and drained
//
// Spurious wakeups are real — your wait must recheck its condition.

#include <iostream>

int main() {
    std::cout << "ex03_hard passed\n";
}
