#include <cassert>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <optional>
#include <queue>

// Exercise: implement IntQueue with push(), pop(), and close().
//
// pop() must block until data arrives OR the queue is closed.
// Always pass a predicate to cv.wait() to handle spurious wakeups.
// close() should wake all blocked pop() callers so they can return nullopt.
// pop() returns nullopt when closed and empty.

int main() {
    // TODO
    std::cout << "ex03_condition_variable passed\n";
}
