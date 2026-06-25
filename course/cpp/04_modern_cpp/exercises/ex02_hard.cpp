// Exercise 02 (Hard) - Lambda Callback Lifetime Bugs
//
// Tasks:
//   1. Implement CallbackRegistry using std::function<void(int)>.
//   2. Demonstrate safe capture-by-value and capture-by-reference cases.
//   3. Explain why storing a lambda that captures a local by reference is risky.
//   4. Ensure emit() invokes callbacks in registration order.
//
// A3 (danger of capture-by-reference in stored lambdas):
//   A lambda that captures local variables by reference stores POINTERS to
//   those locals. If the lambda outlives the scope of those variables (because
//   it was stored in a CallbackRegistry or similar), the stored references
//   dangle. The next emit() call dereferences garbage memory — undefined
//   behaviour. Capture by value is safe for long-lived stored lambdas.
//   Capture by reference is safe for short-lived lambdas (e.g., std::sort
//   comparator within the same scope).

#include <cassert>
#include <functional>
#include <iostream>
#include <vector>

class CallbackRegistry {
public:
    void add(std::function<void(int)> callback) {
        callbacks_.push_back(std::move(callback));
    }

    void emit(int value) const {
        for (const auto& cb : callbacks_) cb(value);
    }

private:
    std::vector<std::function<void(int)>> callbacks_;
};

int main() {
    CallbackRegistry registry;

    int sum = 0;
    int multiplier = 3;

    // Capture sum by reference (safe here — sum outlives registry).
    registry.add([&sum](int v) { sum += v; });

    // Mix: capture multiplier by value (avoids dangling if local goes away),
    // capture sum by reference.
    registry.add([multiplier, &sum](int v) { sum += multiplier * v; });

    registry.emit(2);
    assert(sum == 8);  // 2 + 3*2 = 8

    // Verify order: first callback adds 2, second adds 6, total 8.
    int trace = 0;
    std::vector<int> order;
    CallbackRegistry ordered;
    ordered.add([&order](int v) { order.push_back(v * 10); });
    ordered.add([&order](int v) { order.push_back(v * 100); });
    ordered.emit(1);
    assert(order.size() == 2);
    assert(order[0] == 10);
    assert(order[1] == 100);
    (void)trace;

    std::cout << "ex02_hard passed\n";
}

