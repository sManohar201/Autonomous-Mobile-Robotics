#include <cassert>
#include <iostream>
#include <vector>

std::vector<double> make_ranges(std::size_t n) {
    std::vector<double> ranges;
    ranges.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        ranges.push_back(static_cast<double>(i));
    }
    return ranges;
}

int main() {
    auto ranges = make_ranges(100);
    assert(ranges.size() == 100);
    assert(ranges.capacity() >= 100);
    assert(ranges[42] == 42.0);
    std::cout << "ex02_allocation_answer passed\n";
}

