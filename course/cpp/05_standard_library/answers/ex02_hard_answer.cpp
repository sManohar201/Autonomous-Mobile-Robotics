#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

struct Record { double t; double range; bool valid; };

void clean_and_sort(std::vector<Record>& records) {
    records.erase(std::remove_if(records.begin(), records.end(),
                                 [](const Record& r) { return !r.valid; }),
                  records.end());
    std::stable_sort(records.begin(), records.end(),
                     [](const Record& a, const Record& b) { return a.t < b.t; });
}

std::vector<double> normalized_ranges(const std::vector<Record>& records, double max_range) {
    std::vector<double> out;
    out.reserve(records.size());
    std::transform(records.begin(), records.end(), std::back_inserter(out),
                   [max_range](const Record& r) { return r.range / max_range; });
    return out;
}

double sum_ranges(const std::vector<Record>& records) {
    return std::accumulate(records.begin(), records.end(), 0.0,
                           [](double sum, const Record& r) { return sum + r.range; });
}

int main() {
    std::vector<Record> records{{2, 4, true}, {1, 99, false}, {0, 2, true}};
    clean_and_sort(records);
    assert(records.size() == 2);
    assert(records[0].t == 0.0);
    assert(sum_ranges(records) == 6.0);
    assert(normalized_ranges(records, 4.0)[1] == 1.0);
    std::cout << "ex02_hard_answer passed\n";
}

