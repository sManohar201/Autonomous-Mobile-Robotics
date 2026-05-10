// Exercise 02 (Hard) - Compile-Time Matrix Dimensions
//
// Context:
//   EKF covariance math must reject dimension mismatches at compile time.
//
// Pre-coding questions:
//   Q1. Why should Matrix<double, 2, 3> and Matrix<double, 3, 2> be different
//       types?
//   Q2. Where should static_assert checks live: class, free function, or both?
//
// Tasks:
//   1. Implement Matrix<T, Rows, Cols>.
//   2. Implement matrix multiplication with compile-time dimensions.
//   3. Implement identity<T, N>().
//   4. Demonstrate that invalid dimensions fail to compile when uncommented.

#include <iostream>

int main() {
    std::cout << "ex02_hard passed\n";
}

