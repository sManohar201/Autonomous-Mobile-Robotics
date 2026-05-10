#include <cassert>
#include <iostream>
#include <vector>

int safe_sum(const std::vector<int>& values) {
    int sum = 0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        sum += values[i];
    }
    return sum;
}

int main() {
    std::vector<int> values{1, 2, 3};
    assert(safe_sum(values) == 6);
    std::cout << "ex04_sanitizer_bughunt_answer passed\n";
}

