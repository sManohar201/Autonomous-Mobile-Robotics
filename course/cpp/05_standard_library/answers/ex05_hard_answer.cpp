#include <cassert>
#include <chrono>
#include <deque>
#include <iostream>
#include <optional>

using Clock = std::chrono::steady_clock;

struct Sample { Clock::time_point t; double value; };

class TimeWindow {
public:
    explicit TimeWindow(std::chrono::milliseconds window) : window_{window} {}
    void push(Sample s) { samples_.push_back(s); prune(s.t); }
    void prune(Clock::time_point now) {
        while (!samples_.empty() && now - samples_.front().t > window_) samples_.pop_front();
    }
    double mean() const {
        if (samples_.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& s : samples_) sum += s.value;
        return sum / static_cast<double>(samples_.size());
    }
    std::optional<double> interpolate(Clock::time_point t) const {
        for (std::size_t i = 1; i < samples_.size(); ++i) {
            if (samples_[i - 1].t <= t && t <= samples_[i].t) {
                auto span = std::chrono::duration<double>(samples_[i].t - samples_[i - 1].t).count();
                auto alpha = std::chrono::duration<double>(t - samples_[i - 1].t).count() / span;
                return samples_[i - 1].value + alpha * (samples_[i].value - samples_[i - 1].value);
            }
        }
        return std::nullopt;
    }
private:
    std::chrono::milliseconds window_;
    std::deque<Sample> samples_;
};

int main() {
    auto t0 = Clock::now();
    TimeWindow w{std::chrono::milliseconds{100}};
    w.push({t0, 0.0});
    w.push({t0 + std::chrono::milliseconds{100}, 10.0});
    assert(w.mean() == 5.0);
    assert(w.interpolate(t0 + std::chrono::milliseconds{50}).value() == 5.0);
    std::cout << "ex05_hard_answer passed\n";
}
