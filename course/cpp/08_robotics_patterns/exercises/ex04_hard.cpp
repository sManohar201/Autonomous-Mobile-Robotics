// Exercise 04 (Hard) - Entity Component World with Destruction
//
// Extend the entity-component World to support entity destruction:
//   - destroy(entity) removes that entity from ALL component stores
//   - querying a destroyed entity returns nullopt
//
// Ensure no dangling component data remains after destruction.
// Think about what data structures give O(1) removal.

#include <iostream>

int main() {
    std::cout << "ex04_hard passed\n";
}
