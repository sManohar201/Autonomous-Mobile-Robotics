#include <cassert>
#include <iostream>
#include <vector>

// Exercise: implement make_ranges(n) returning a vector<double> of 0.0 to n-1.0.
//
// Reserve capacity before pushing to avoid repeated heap reallocations.
// Without reserve, the vector grows geometrically — O(log n) allocations.
// With reserve, exactly one allocation covers all n elements.

int main() {
    // TODO
    std::cout << "ex02_allocation passed\n";
}
