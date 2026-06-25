#include <cassert>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

// Exercise: implement ThreadSafeCounter with increment() and value().
//
// Both methods must lock the same mutex.
// value() is const, so the mutex must be declared mutable.
// Use std::lock_guard for RAII locking — never unlock manually.
//
// Test: 4 threads each call increment() 1000 times → final value == 4000

int main() {
    // TODO
    std::cout << "ex02_mutex passed\n";
}
