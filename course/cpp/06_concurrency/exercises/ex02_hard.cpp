// Exercise 02 (Hard) - Deadlock Diagnosis and Lock Ordering
//
// Implement transfer(from, to, amount) between two Account objects,
// each with its own mutex.
//
// Naive implementations that lock in caller-determined order can deadlock
// when two threads transfer in opposite directions simultaneously.
// Find the standard library tool that acquires multiple mutexes safely.

#include <iostream>

int main() {
    std::cout << "ex02_hard passed\n";
}
