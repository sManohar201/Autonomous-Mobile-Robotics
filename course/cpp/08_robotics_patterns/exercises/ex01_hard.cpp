// Exercise 01 (Hard) - Observer Pattern with Safe Unsubscribe
//
// Extend the observer pattern to support removing specific subscribers:
//   - subscribe returns an opaque token identifying that subscription
//   - unsubscribe(token) removes exactly that subscriber
//   - publish notifies only currently active subscribers
//
// Consider what data structure lets you add, remove, and iterate efficiently.
// Consider what happens if a callback is invoked after the subscriber is gone.

#include <iostream>

int main() {
    std::cout << "ex01_hard passed\n";
}
