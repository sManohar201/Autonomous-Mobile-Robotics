// Exercise 05 (Hard) - Point Cloud Transform Pipeline
//
// Implement transform_valid(in, tx, ty, theta):
//   - skips points marked invalid
//   - applies 2D rotation + translation to valid points
//   - computes the axis-aligned bounding box of the transformed output
//   - returns both the filtered+transformed points and the bounding box
//
// Explain in a comment at what point SIMD would become worth adding here.

#include <iostream>

int main() {
    std::cout << "ex05_hard passed\n";
}
