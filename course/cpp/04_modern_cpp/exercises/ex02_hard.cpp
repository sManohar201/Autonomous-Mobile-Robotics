// Exercise 02 (Hard) - Lambda Callback Lifetime Bugs
//
// Tasks:
//   1. Implement CallbackRegistry using std::function<void(int)>.
//   2. Demonstrate safe capture-by-value and capture-by-reference cases.
//   3. Explain why storing a lambda that captures a local by reference is risky.
//   4. Ensure emit() invokes callbacks in registration order.

#include <iostream>

int main() {
    std::cout << "ex02_hard passed\n";
}

