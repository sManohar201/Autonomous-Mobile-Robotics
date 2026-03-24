// ANSWER — hypotenuse.cpp (Task 3 of Exercise 01)
// ─────────────────────────────────────────────────────────────────────────────
// This is the separate translation unit for Task 3.
// It compiles to hypotenuse.o. The linker combines it with ex01_pipeline.o.
// No #include of main — the two files are independent.
// ─────────────────────────────────────────────────────────────────────────────

#include <cmath>

double compute_hypotenuse(double a, double b) {
    return std::sqrt(a*a + b*b);
}
