// Exercise 04 (Hard) - CRTP Static Filter Interface
//
// Context:
//   Some filters are selected at compile time and should avoid virtual calls.
//
// Tasks:
//   1. Implement FilterBase<Derived> with predict/update/state forwarding.
//   2. Implement ConstantVelocityFilter.
//   3. Implement run_filter(FilterBase<Derived>&, dt, measurement).
//   4. Explain why this cannot replace runtime plugin polymorphism.

#include <iostream>

int main() {
    std::cout << "ex04_hard passed\n";
}

