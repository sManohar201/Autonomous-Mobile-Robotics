#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

struct Reading {
    double range_m;
    bool valid;
};

std::size_t count_valid(const std::vector<Reading>& readings) {
    return static_cast<std::size_t>(std::count_if(
        readings.begin(), readings.end(),
        [](const Reading& r) { return r.valid; }));
}

void remove_invalid(std::vector<Reading>& readings) {
    readings.erase(std::remove_if(readings.begin(), readings.end(),
                                  [](const Reading& r) { return !r.valid; }),
                   readings.end());
}

std::vector<double> squared_ranges(const std::vector<Reading>& readings) {
    std::vector<double> out;
    out.reserve(readings.size());
    std::transform(readings.begin(), readings.end(), std::back_inserter(out),
                   [](const Reading& r) { return r.range_m * r.range_m; });
    return out;
}

double sum_ranges(const std::vector<Reading>& readings) {
    return std::accumulate(readings.begin(), readings.end(), 0.0,
                           [](double sum, const Reading& r) {
                               return sum + r.range_m;
                           });
}

int main() {
    std::vector<Reading> readings{{1.0, true}, {2.0, false}, {3.0, true}};
    assert(count_valid(readings) == 2);
    assert(sum_ranges(readings) == 6.0);
    auto sq = squared_ranges(readings);
    assert(sq[2] == 9.0);
    remove_invalid(readings);
    assert(readings.size() == 2);
    std::cout << "ex02_algorithms_answer passed\n";
}

