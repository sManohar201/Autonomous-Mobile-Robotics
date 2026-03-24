// ANSWER — Exercise 02: Types, const, and const-correctness
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <cstdint>
#include <cmath>

// TODO 1: constexpr double, not #define.
//
// #define GRAVITY 9.81  — preprocessor text substitution, no type, no scope.
// constexpr double      — real type (double), compiler-enforced read-only,
//                         visible in debugger, respects scope rules.
constexpr double GRAVITY = 9.81;

double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// TODO 2: const on value parameters prevents accidental re-assignment inside
// the function. More meaningful when parameters are references — const& prevents
// modification of the caller's variable through the reference.
double clamp(double value, const double lo, const double hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

// TODO 3: const double* p — pointer to const double.
// Reading right-to-left: "p is a pointer to const double".
// *p cannot be modified. p itself can point elsewhere (not needed here).
void print_value(const double* p) {
    // *p = 999.0;  ← compile error: assignment of read-only location
    std::cout << "value: " << *p << "\n";
}

int main() {
    std::cout << "distance: " << distance(0, 0, 3, 4) << "\n";
    std::cout << "clamped: "  << clamp(1.5, 0.0, 1.0) << "\n";

    // TODO 4: uint8_t holds 0–255. Assigning 300 wraps: 300 % 256 = 44.
    // The compiler warns: "changes value from 300 to 44".
    // Silent in release builds — a real source of bugs in embedded/sensor code.
    uint8_t b = 300;
    std::cout << "byte wrapped: " << static_cast<int>(b) << "\n";

    // TODO 5: constexpr variables are read-only after initialisation.
    // Error: "assignment of read-only variable 'GRAVITY'"
    // GRAVITY = 10.0;

    // TODO 6: &val gives double*. Passing to const double* is legal —
    // you can always add const, never implicitly remove it.
    double val = 3.14;
    print_value(&val);

    return 0;
}
