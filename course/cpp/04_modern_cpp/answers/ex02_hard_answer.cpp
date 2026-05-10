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
        for (const auto& callback : callbacks_) callback(value);
    }

private:
    std::vector<std::function<void(int)>> callbacks_;
};

int main() {
    CallbackRegistry registry;
    int sum = 0;
    int multiplier = 3;
    registry.add([&sum](int v) { sum += v; });
    registry.add([multiplier, &sum](int v) { sum += multiplier * v; });
    registry.emit(2);
    assert(sum == 8);
    std::cout << "ex02_hard_answer passed\n";
}

