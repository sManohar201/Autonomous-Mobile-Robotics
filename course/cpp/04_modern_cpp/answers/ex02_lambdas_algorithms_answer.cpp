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

std::size_t count_valid(const std::vector<RangeReading>& readings) {
    return static_cast<std::size_t>(std::count_if(
        readings.begin(), readings.end(),
        [](const RangeReading& r) { return r.valid; }));
}

double average_valid_range(const std::vector<RangeReading>& readings) {
    struct Accumulator {
        double sum{};
        std::size_t count{};
    };

    const auto acc = std::accumulate(
        readings.begin(), readings.end(), Accumulator{},
        [](Accumulator current, const RangeReading& r) {
            if (r.valid) {
                current.sum += r.range_m;
                ++current.count;
            }
            return current;
        });

    return acc.count == 0 ? 0.0 : acc.sum / static_cast<double>(acc.count);
}

class RangeProcessor {
public:
    using Callback = std::function<void(double)>;

    explicit RangeProcessor(Callback callback)
        : callback_{std::move(callback)}
    {}

    void process(const std::vector<RangeReading>& readings) const {
        callback_(average_valid_range(readings));
    }

private:
    Callback callback_;
};

int main() {
    std::vector<RangeReading> readings{
        {1.0, true},
        {100.0, false},
        {3.0, true},
    };

    assert(count_valid(readings) == 2);
    assert(average_valid_range(readings) == 2.0);

    double last_average = 0.0;
    RangeProcessor processor{[&last_average](double avg) {
        last_average = avg;
    }};
    processor.process(readings);
    assert(last_average == 2.0);

    std::cout << "ex02_lambdas_algorithms_answer passed\n";
}

