#include <cassert>
#include <iostream>
#include <vector>

std::size_t capacity_growths(std::size_t n, bool reserve_first) {
    std::vector<int> values;
    if (reserve_first) values.reserve(n);
    std::size_t growths = 0;
    std::size_t cap = values.capacity();
    for (std::size_t i = 0; i < n; ++i) {
        values.push_back(static_cast<int>(i));
        if (values.capacity() != cap) {
            cap = values.capacity();
            ++growths;
        }
    }
    return growths;
}

int main() {
    assert(capacity_growths(100, true) == 0);
    assert(capacity_growths(100, false) > 0);
    std::cout << "ex02_hard_answer passed\n";
}

