// Exercise 02 - Lambdas, Algorithms, and std::function
//
// Goal:
//   Replace low-level loops with clear standard algorithms and use callbacks
//   at the right boundary.

#include <algorithm>
#include <cassert>
#include <functional>
#include <iostream>
#include <numeric>
#include <vector>

struct RangeReading {
    double range_m;
    bool valid;
};

// TODO 1: Implement count_valid using std::count_if.

// TODO 2: Implement average_valid_range using std::accumulate or a clear loop.
// Return 0.0 if there are no valid readings.

class RangeProcessor {
public:
    using Callback = std::function<void(double)>;

    // TODO 3: Constructor accepts Callback and stores it.

    // TODO 4: process(readings) computes average valid range and calls callback.

private:
    // TODO: store callback.
};

int main() {
    std::vector<RangeReading> readings{
        {1.0, true},
        {100.0, false},
        {3.0, true},
    };

    // TODO: make these pass.
    // assert(count_valid(readings) == 2);
    // assert(average_valid_range(readings) == 2.0);
    //
    // double last_average = 0.0;
    // RangeProcessor processor{[&last_average](double avg) {
    //     last_average = avg;
    // }};
    // processor.process(readings);
    // assert(last_average == 2.0);

    (void)readings;

    std::cout << "ex02_lambdas_algorithms passed\n";
}

