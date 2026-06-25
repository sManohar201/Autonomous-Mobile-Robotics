// Exercise 04 (Hard) - Fold Expressions for Validation Frameworks
//
// Tasks:
//   1. Implement all_of_values and any_of_values with folds.
//   2. Implement validate_all(checks...) where checks are callables.
//   3. Implement sum_sizes(containers...) using a fold.
//   4. Explain the empty-pack identity for && and || folds.
//
// A4 (empty-pack identity for && and || folds):
//   When a fold expression has no arguments (empty pack), C++ requires a
//   well-defined identity value so the expression has a result.
//   For `(args && ...)`: empty pack → true  (identity of AND: true && x = x)
//   For `(args || ...)`: empty pack → false (identity of OR:  false || x = x)
//   For `(args + ...)` over integers: empty pack → 0
//   If the type has no natural identity, use `(init op ... op pack)` (binary
//   fold) to provide an explicit initial value.

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
    assert(!any_of_values(false, false));

    assert(validate_all([] { return true; }, [] { return 2 + 2 == 4; }));
    assert(!validate_all([] { return true; }, [] { return false; }));

    std::vector<int> a{1, 2};
    std::string b{"abc"};
    assert(sum_sizes(a, b) == 5);

    // Empty pack identities:
    assert(all_of_values() == true);
    assert(any_of_values() == false);

    std::cout << "ex04_hard passed\n";
}

