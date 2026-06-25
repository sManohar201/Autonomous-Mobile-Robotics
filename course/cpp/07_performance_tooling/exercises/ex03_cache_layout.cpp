#include <cassert>
#include <iostream>
#include <vector>

struct Point { float x, y, z; };

// Exercise: implement sum_z over a vector<Point>.
//
// vector<Point> stores structs contiguously (AoS layout).
// Sequential iteration touches each cache line once — that's cache-friendly.
// Why? The CPU prefetches ahead in a linear access pattern with no gaps.

int main() {
    // TODO
    std::cout << "ex03_cache_layout passed\n";
}
