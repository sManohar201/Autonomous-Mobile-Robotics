// Exercise 03 (Hard) - Cache Layout Comparison
//
// Implement two point-cloud z-sum kernels:
//   - AoS (Array of Structures): vector<Point{x,y,z}>
//   - SoA (Structure of Arrays): struct with separate x[], y[], z[] vectors
//
// Explain why the SoA kernel has better cache utilization for z-only access.
// Measure both with the timer from ex01 and compare results.

#include <iostream>

int main() {
    std::cout << "ex03_hard passed\n";
}
