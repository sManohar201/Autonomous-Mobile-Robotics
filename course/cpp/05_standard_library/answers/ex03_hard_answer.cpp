#include <cassert>
#include <chrono>
#include <deque>
#include <iostream>

using Clock = std::chrono::steady_clock;

class Deadline {
public:
    Deadline(Clock::time_point start, std::chrono::milliseconds timeout)
        : expires_at_{start + timeout} {}
    bool expired(Clock::time_point now) const { return now >= expires_at_; }
private:
    Clock::time_point expires_at_;
};

struct Sample { Clock::time_point t; double value; };

void prune_old(std::deque<Sample>& samples, Clock::time_point now, std::chrono::milliseconds window) {
    while (!samples.empty() && now - samples.front().t > window) samples.pop_front();
}

int main() {
    auto t0 = Clock::now();
    Deadline d{t0, std::chrono::milliseconds{100}};
    assert(!d.expired(t0 + std::chrono::milliseconds{99}));
    assert(d.expired(t0 + std::chrono::milliseconds{100}));
    std::deque<Sample> samples{{t0, 1.0}, {t0 + std::chrono::milliseconds{50}, 2.0}};
    prune_old(samples, t0 + std::chrono::milliseconds{110}, std::chrono::milliseconds{60});
    assert(samples.size() == 1);
    std::cout << "ex03_hard_answer passed\n";
}
