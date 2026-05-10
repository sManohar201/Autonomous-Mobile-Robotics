#include <cassert>
#include <chrono>
#include <deque>
#include <iostream>

struct Sample {
    std::chrono::steady_clock::time_point stamp;
    double value;
};

// TODO: implement SensorWindow with:
//   - explicit constructor(std::chrono::milliseconds window)
//   - push(Sample)
//   - evict(now)
//   - size()
//   - mean()
// Store samples in std::deque.

int main() {
    std::cout << "ex05_capstone_sensor_window passed\n";
}

