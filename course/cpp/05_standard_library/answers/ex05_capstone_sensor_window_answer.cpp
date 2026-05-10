#include <cassert>
#include <chrono>
#include <deque>
#include <iostream>

struct Sample {
    std::chrono::steady_clock::time_point stamp;
    double value;
};

class SensorWindow {
public:
    explicit SensorWindow(std::chrono::milliseconds window)
        : window_{window}
    {}

    void push(Sample sample) {
        samples_.push_back(sample);
        evict(sample.stamp);
    }

    void evict(std::chrono::steady_clock::time_point now) {
        while (!samples_.empty() && now - samples_.front().stamp > window_) {
            samples_.pop_front();
        }
    }

    std::size_t size() const { return samples_.size(); }

    double mean() const {
        if (samples_.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& sample : samples_) sum += sample.value;
        return sum / static_cast<double>(samples_.size());
    }

private:
    std::chrono::milliseconds window_;
    std::deque<Sample> samples_;
};

int main() {
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();
    SensorWindow window{std::chrono::milliseconds{100}};
    window.push({t0, 1.0});
    window.push({t0 + std::chrono::milliseconds{50}, 3.0});
    assert(window.size() == 2);
    assert(window.mean() == 2.0);
    window.push({t0 + std::chrono::milliseconds{150}, 5.0});
    assert(window.size() == 2);
    assert(window.mean() == 4.0);
    std::cout << "ex05_capstone_sensor_window_answer passed\n";
}
