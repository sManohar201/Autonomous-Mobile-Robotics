#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>

// TODO: implement BoundedBlockingQueue<T>:
//   - constructor(capacity)
//   - push(T) returns false if closed
//   - pop() returns optional<T>; nullopt means closed and empty
//   - close()
//   - size()
// If full, push should block until space exists or closed.

int main() {
    std::cout << "ex05_capstone_blocking_queue passed\n";
}

