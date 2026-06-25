// Exercise 04 (Hard) - Atomics vs Mutexes
//
// Implement two things and explain when to use each:
//   1. An atomic counter — a standalone integer modified by multiple threads
//   2. A thread-safe sample store — a class wrapping a std::vector
//
// The core question: why can't you use atomic<bool> to protect a vector?
// What property does the vector lack that a plain integer has?

#include <iostream>

int main() {
    std::cout << "ex04_hard passed\n";
}
