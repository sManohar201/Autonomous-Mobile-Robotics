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

template <typename T>
T clamp(T value, T lo, T hi) {
    if (value < lo) return lo;
    if (hi < value) return hi;
    return value;
}

template <typename T>
T square(T x) {
    return x * x;
}

// Accepts only arithmetic types — the static_assert fires at instantiation for
// pointer, struct, or other non-arithmetic T.
template <typename T>
double magnitude2(T x, T y) {
    static_assert(std::is_arithmetic_v<T>, "magnitude2 requires arithmetic T");
    const double dx = static_cast<double>(x);
    const double dy = static_cast<double>(y);
    return std::sqrt(dx * dx + dy * dy);
}

// Q4 answer — Why does max_same_type(1, 2.5) fail with one template param T?
//   Template argument deduction deduces T from each argument independently.
//   From `1` → T = int.  From `2.5` → T = double.
//   The two deductions conflict; the compiler cannot resolve T to a single type.
//   It does NOT perform implicit conversions during deduction — deduction must
//   succeed unambiguously before any conversion is applied.
//   Fix: explicit instantiation max_same_type<double>(1, 2.5), or use two params.
template <typename T>
T max_same_type(T a, T b) {
    return a < b ? b : a;
}

int main() {
    assert(clamp(9, 0, 5) == 5);
    assert(clamp(-1.0, 0.0, 10.0) == 0.0);
    assert(square(4) == 16);
    assert(std::abs(magnitude2(3, 4) - 5.0) < 1e-9);

    auto m = max_same_type<double>(1, 2.5);
    assert(m == 2.5);

    std::cout << "ex01_function_templates passed\n";
}
