// Exercise 02 — Types, const, and const-correctness
//
// TASK:
//   Several lines below are marked TODO. Complete each one as instructed.
//
// EXPECTED OUTPUT:
//   distance: 5
//   clamped: 1
//   byte wrapped: 44
//   value: 3.14

#include <iostream>
#include <cstdint>
#include <cmath>

// TODO 1: Declare a constexpr double named GRAVITY with value 9.81
constexpr double GRAVITY = 9.81;

double distance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx*dx + dy*dy);
}

// TODO 2: The parameters lo and hi are not modified inside this function.
// Add the correct const qualifier to both.
double clamp(double value, const double lo, const double hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

// TODO 3: This function must NOT modify the value pointed to by p.
// Add the correct const qualifier to the parameter.
void print_value(const double* p) {
    std::cout << "value: " << *p << "\n";
}

int main() {
    std::cout << "distance: " << distance(0, 0, 3, 4) << "\n";
    std::cout << "clamped: "  << clamp(1.5, 0.0, 1.0) << "\n";

    // TODO 4: Predict what this prints before running. Why does it wrap?
    // ANSWER: 300 - 256 = 44. uint8_t can only hold 0-255.
    // 300 in binary is 100101100; truncated to 8 bits = 00101100 = 44.
    uint8_t b = 300;
    std::cout << "byte wrapped: " << static_cast<int>(b) << "\n";

    // TODO 5: Uncomment the line below — it should NOT compile.
    // WHY IT FAILS: GRAVITY is declared constexpr, which implies const.
    // Attempting to assign to a const variable is a compile error.
    // GRAVITY = 10.0;

    // TODO 6: Call print_value with the address of val.
    double val = 3.14;
    print_value(&val);

    return 0;
}
