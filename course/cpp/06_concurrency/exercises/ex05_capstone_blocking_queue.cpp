#include <cassert>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <iostream>
#include <mutex>
#include <optional>
#include <thread>

// Exercise: implement BoundedBlockingQueue<T> with capacity limiting.
//
// push(T)  — blocks when full; returns false immediately if closed
// pop()    — blocks when empty; returns nullopt when closed and drained
// close()  — wakes all blocked callers on both push and pop sides
// size()   — returns current element count
//
// Use two condition variables (not_empty_ and not_full_) to avoid
// waking the wrong side unnecessarily.

int main() {
    // TODO
    std::cout << "ex05_capstone_blocking_queue passed\n";
}
