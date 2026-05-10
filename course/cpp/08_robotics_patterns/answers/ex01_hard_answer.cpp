#include <cassert>
#include <functional>
#include <iostream>
#include <unordered_map>

class EventBus {
public:
    using Token = int;
    using Callback = std::function<void(int)>;
    Token subscribe(Callback cb) {
        callbacks_[next_] = std::move(cb);
        return next_++;
    }
    bool unsubscribe(Token token) {
        return callbacks_.erase(token) == 1;
    }
    void publish(int value) const {
        for (const auto& [token, cb] : callbacks_) {
            (void)token;
            cb(value);
        }
    }
private:
    Token next_{1};
    std::unordered_map<Token, Callback> callbacks_;
};

int main() {
    EventBus bus;
    int sum = 0;
    auto token = bus.subscribe([&](int v) { sum += v; });
    bus.publish(3);
    assert(sum == 3);
    assert(bus.unsubscribe(token));
    bus.publish(3);
    assert(sum == 3);
    std::cout << "ex01_hard_answer passed\n";
}

