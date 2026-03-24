// Exercise 01 — The Compilation Pipeline
//
// TASK:
//   This file deliberately contains a declaration-only call to a function
//   that is NOT defined anywhere. Your job is to:
//
//   1. Build this as-is and observe the error. Is it a compiler error or a
//      linker error? How can you tell from the message?
//
//   2. Add the definition of `compute_hypotenuse` in this same file and
//      rebuild. It should pass both compilation and linking.
//
//   3. (Challenge) Move the definition into a separate file `hypotenuse.cpp`
//      and update CMakeLists.txt to compile both files into the executable.
//      Verify the linker error returns when you forget to add the second file.
//
// EXPECTED OUTPUT (after fix):
//   hypotenuse(3, 4) = 5
//   hypotenuse(5, 12) = 13

#include <iostream>
#include <cmath>

// Declaration only — the definition is missing.
// This tells the compiler the function exists. The linker must find the body.
double compute_hypotenuse(double a, double b);

// ─── ADD YOUR DEFINITION HERE (Task 2) ───────────────────────────────────────



// ─────────────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "hypotenuse(3, 4) = "  << compute_hypotenuse(3.0, 4.0)  << "\n";
    std::cout << "hypotenuse(5, 12) = " << compute_hypotenuse(5.0, 12.0) << "\n";
    return 0;
}
