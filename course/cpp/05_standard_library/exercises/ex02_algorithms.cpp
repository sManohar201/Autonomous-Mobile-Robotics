#include <algorithm>
#include <cassert>
#include <iostream>
#include <numeric>
#include <vector>

struct Reading {
    double range_m;
    bool valid;
};

// TODO: count valid readings with std::count_if.
// TODO: remove invalid readings with erase-remove.
// TODO: transform ranges into squared ranges.
// TODO: sum ranges with std::accumulate.

int main() {
    std::cout << "ex02_algorithms passed\n";
}

