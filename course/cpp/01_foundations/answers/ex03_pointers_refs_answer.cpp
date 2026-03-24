// ANSWER — Exercise 03: Pointers and References
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <cmath>

// 1. Swap using references.
// References are aliases — writing to a or b directly modifies caller's memory.
void swap_by_ref(int& a, int& b) {
    int temp = a;
    a = b;
    b = temp;
}

// 2. Swap using pointers.
//
// COMMON BUG: saving *b instead of *a.
//   int temp = *b;  *a = *b;  *b = temp;
//   After *a = *b: both locations hold b's value. temp = b's original value.
//   *b = temp restores b to itself. Net result: both become b's original value.
//
// The rule: save the value you are about to OVERWRITE — which is *a.
void swap_by_ptr(int* a, int* b) {
    int temp = *a;
    *a = *b;
    *b = temp;
}

// 3. Normalize in-place.
// Compute magnitude BEFORE modifying x or y — modifying x first would give
// the wrong magnitude on the second computation.
void normalize(double& x, double& y) {
    double mag = std::sqrt(x*x + y*y);
    x = x / mag;
    y = y / mag;
}

// 4. safe_divide with output pointer.
//
// Why a pointer (not a reference) for result?
//   A reference cannot be null. A pointer allows the caller to pass nullptr
//   when they don't need the output value.
//
// Why b != 0.0 and not b > 0?
//   Negative denominators are valid: 5 / -2 = -2.5.
//   b > 0 silently skips division for negative b — wrong behaviour.
void safe_divide(double a, double b, double* result) {
    if (b != 0.0) {
        if (result) *result = a / b;
    } else {
        std::cout << "Divide by zero\n";
    }
}

int main() {
    int a = 10, b = 20;
    swap_by_ref(a, b);
    std::cout << "swap_by_ref: a=" << a << " b=" << b << "\n";

    int x = 99, y = 5;
    swap_by_ptr(&x, &y);
    std::cout << "swap_by_ptr: a=" << x << " b=" << y << "\n";

    double vx = 3.0, vy = 4.0;
    normalize(vx, vy);
    std::cout << "normalize in-place: x=" << vx << " y=" << vy << "\n";

    double out;
    safe_divide(5.0, 2.0, &out);
    std::cout << "safe_divide: " << out << "\n";

    safe_divide(5.0, 0.0, &out);
    return 0;
}
