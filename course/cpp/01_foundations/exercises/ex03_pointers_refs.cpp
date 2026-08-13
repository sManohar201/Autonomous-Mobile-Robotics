// Exercise 03 — Pointers and References
//
// TASK:
//   Implement the four functions below using the technique specified in each.
//   Do not change the function signatures.
//
// EXPECTED OUTPUT:
//   swap_by_ref: a=20 b=10
//   swap_by_ptr: a=5 b=99
//   normalize in-place: x=0.6 y=0.8
//   safe_divide: 2.5
//   Divide by zero

#include <iostream>
#include <cmath>

// 1. Swap two integers using references.
//    After the call: a and b should have exchanged values.
void swap_by_ref(int& a, int& b) {
    // TODO
}

// 2. Swap two integers using pointers.
//    After the call: *a and *b should have exchanged values.
void swap_by_ptr(int* a, int* b) {
    // TODO
}

// 3. Normalise a 2D vector in-place using references.
//    After the call: (x,y) should have magnitude 1.0.
void normalize(double& x, double& y) {
    // TODO
}

// 4. Divide a by b. Store the result in *result.
//    If b is zero: print "Divide by zero" and return without touching *result.
//    Use a pointer for result — null means "caller doesn't need the output".
void safe_divide(double a, double b, double* result) {
    // TODO
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
