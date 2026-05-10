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

template <typename T>
double magnitude2(T x, T y) {
    static_assert(std::is_arithmetic_v<T>, "magnitude2 requires arithmetic T");
    const double dx = static_cast<double>(x);
    const double dy = static_cast<double>(y);
    return std::sqrt(dx * dx + dy * dy);
}

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

    std::cout << "ex01_function_templates_answer passed\n";
}

