#include <cassert>
#include <iostream>
#include <thread>
#include <vector>

// Exercise: launch N threads where thread i writes values[i] = i, then join all.
//
// Steps:
//   1. Create a vector<int> of size N pre-filled with -1
//   2. Spawn N threads — capture i by VALUE (not reference) to avoid a race
//   3. Join every thread before reading the vector
//
// Hint: use threads.reserve(n) before emplace_back to avoid iterator invalidation

int main() {
    // TODO
    std::cout << "ex01_threads passed\n";
}
