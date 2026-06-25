// Exercise 01 (Hard) - Thread Lifecycle and Cooperative Shutdown
//
// Design and implement a Worker class that:
//   - launches a background thread in its constructor
//   - loops until a stop signal is received
//   - stops cleanly and joins in its destructor
//   - exposes a counter showing how many loop iterations ran
//
// Consider: what happens to a joinable thread that is destroyed without joining?

#include <iostream>

int main() {
    std::cout << "ex01_hard passed\n";
}
