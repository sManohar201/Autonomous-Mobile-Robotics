#include <cassert>
#include <chrono>
#include <iostream>

using Clock = std::chrono::steady_clock;

bool expired(Clock::time_point start,
             Clock::time_point now,
             std::chrono::milliseconds timeout) {
    return now - start >= timeout;
}

int main() {
    auto start = Clock::now();
    assert(!expired(start, start + std::chrono::milliseconds{50},
                    std::chrono::milliseconds{100}));
    assert(expired(start, start + std::chrono::milliseconds{100},
                   std::chrono::milliseconds{100}));
    std::cout << "ex03_chrono_answer passed\n";
}

