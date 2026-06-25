#include <iostream>

// Exercise: implement a constexpr lifecycle state machine.
//
// States:  Unconfigured → Inactive → Active → (back to Inactive)
//          Any state → Fault (on Error event)
//          Fault → Unconfigured (on Reset)
//
// Define LifecycleState and LifecycleEvent as enum classes.
// transition(state, event) must be constexpr so it can be verified at compile time.
// Unknown/illegal event+state combinations leave the state unchanged.

int main() {
    // TODO
    std::cout << "ex03_fsm passed\n";
}
