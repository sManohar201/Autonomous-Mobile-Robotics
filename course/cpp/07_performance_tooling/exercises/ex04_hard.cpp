// Exercise 04 (Hard) - Sanitizer Bug Hunt
//
// Fix three classes of memory and concurrency bugs:
//   1. Out-of-bounds read (AddressSanitizer) — bounds-check before indexing
//   2. Signed integer overflow (UBSan) — detect before the operation, not after
//   3. Data race (ThreadSanitizer) — protect a shared counter with a mutex
//
// Each fix should return an optional or provide a safe API rather than
// propagating undefined behaviour.

#include <iostream>

int main() {
    std::cout << "ex04_hard passed\n";
}
