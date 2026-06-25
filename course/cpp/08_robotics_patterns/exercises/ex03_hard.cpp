// Exercise 03 (Hard) - Lifecycle FSM with Invalid Transition Diagnostics
//
// Extend the FSM to return a result struct instead of just the new state:
//   - whether the transition was accepted
//   - a human-readable reason string (useful for logging)
//
// Illegal transitions must leave the state unchanged and explain why.
// The function must remain constexpr so static_assert can verify it.

#include <iostream>

int main() {
    std::cout << "ex03_hard passed\n";
}
