#include <atomic>
#include <cassert>
#include <future>
#include <iostream>

// Exercise: demonstrate atomic<bool> as a stop flag and std::async for async work.
//
// Part 1 — atomic stop flag:
//   Create atomic<bool> stop{false}, store true, assert it reads back true.
//
// Part 2 — async sum:
//   Implement sum_to(n) returning 1+2+...+n.
//   Launch it with std::async(std::launch::async, ...) and assert the result is 5050.

int main() {
    // TODO
    std::cout << "ex04_atomic_future passed\n";
}
