#include <cassert>
#include <functional>
#include <iostream>
#include <vector>

struct ImuSample {
    double ax, ay, az;
};

class SensorBus {
public:
    using Callback = std::function<void(const ImuSample&)>;

    void subscribe(Callback callback) {
        callbacks_.push_back(std::move(callback));
    }

    void publish(const ImuSample& sample) const {
        for (const auto& callback : callbacks_) {
            callback(sample);
        }
    }

private:
    std::vector<Callback> callbacks_;
};

int main() {
    SensorBus bus;
    double last_az = 0.0;
    int count = 0;
    bus.subscribe([&](const ImuSample& s) { last_az = s.az; });
    bus.subscribe([&](const ImuSample&) { ++count; });
    bus.publish({0.0, 0.0, 9.8});
    assert(last_az == 9.8);
    assert(count == 1);
    std::cout << "ex01_observer_answer passed\n";
}

