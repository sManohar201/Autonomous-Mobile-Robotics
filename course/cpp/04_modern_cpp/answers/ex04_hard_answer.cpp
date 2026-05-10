#include <cassert>
#include <iostream>
#include <string>
#include <vector>

template <typename... Values>
bool all_of_values(Values... values) {
    return (values && ...);
}

template <typename... Values>
bool any_of_values(Values... values) {
    return (values || ...);
}

template <typename... Checks>
bool validate_all(Checks&&... checks) {
    return (checks() && ...);
}

template <typename... Containers>
std::size_t sum_sizes(const Containers&... containers) {
    return (containers.size() + ... + std::size_t{0});
}

int main() {
    assert(all_of_values(true, true));
    assert(!all_of_values(true, false));
    assert(any_of_values(false, true));
    assert(validate_all([] { return true; }, [] { return 2 + 2 == 4; }));
    std::vector<int> a{1, 2};
    std::string b{"abc"};
    assert(sum_sizes(a, b) == 5);
    std::cout << "ex04_hard_answer passed\n";
}

