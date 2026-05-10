// Exercise 01 - Function Templates
//
// Goal:
//   Implement small generic numeric utilities and reason about template
//   argument deduction.
//
// Build:
//   cmake -B build && cmake --build build
//   ./build/ex01_function_templates

#include <cassert>
#include <cmath>
#include <iostream>
#include <type_traits>

// TODO 1: Implement clamp(T value, T lo, T hi).
// Requirements:
//   - Works for int, float, double.
//   - Does not use std::clamp.

// TODO 2: Implement square(T x).

// TODO 3: Implement magnitude2(x, y).
// Requirements:
//   - Accepts arithmetic types only.
//   - Returns double.
//   - Uses static_assert for the arithmetic constraint.

// TODO 4: Explain before coding:
//   Why does max_same_type(1, 2.5) fail if the function has one template
//   parameter T?
//   YOUR ANSWER:
//   TODO

template <typename T>
T max_same_type(T a, T b) {
    return a < b ? b : a;
}

int main() {
    // TODO: make these pass.
    // assert(clamp(9, 0, 5) == 5);
    // assert(clamp(-1.0, 0.0, 10.0) == 0.0);
    // assert(square(4) == 16);
    // assert(std::abs(magnitude2(3, 4) - 5.0) < 1e-9);

    auto m = max_same_type<double>(1, 2.5);
    assert(m == 2.5);

    std::cout << "ex01_function_templates passed\n";
}

