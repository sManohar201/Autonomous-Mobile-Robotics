// Exercise 02 (Hard) - Compile-Time Matrix Dimensions
//
// Context:
//   EKF covariance math must reject dimension mismatches at compile time.
//
// Pre-coding questions:
//   Q1. Why should Matrix<double, 2, 3> and Matrix<double, 3, 2> be different
//       types?
//   A1. They have different shapes. Allowing implicit conversion between them
//       would let you multiply (2×3)*(2×3) which is undefined — the inner
//       dimensions don't match. Making them distinct types means the compiler
//       rejects invalid operations via template argument deduction, not at
//       runtime. This moves errors from test time to compile time.
//
//   Q2. Where should static_assert checks live: class, free function, or both?
//   A2. For multiply<R,K,C>, the check (inner dimensions match) naturally lives
//       in the free function signature — the types themselves encode the
//       constraint (Matrix<T,R,K> * Matrix<T,K,C> requires the same K).
//       The compiler rejects mismatches automatically via deduction failure.
//       static_assert in the class is useful for preconditions like Rows > 0.
//
// Tasks:
//   1. Implement Matrix<T, Rows, Cols>.
//   2. Implement matrix multiplication with compile-time dimensions.
//   3. Implement identity<T, N>().
//   4. Demonstrate that invalid dimensions fail to compile when uncommented.

#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>

template <typename T, std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    static_assert(Rows > 0 && Cols > 0, "Matrix dimensions must be positive");

    T& operator()(std::size_t r, std::size_t c) {
        return data_[r * Cols + c];
    }
    const T& operator()(std::size_t r, std::size_t c) const {
        return data_[r * Cols + c];
    }

    constexpr std::size_t rows() const { return Rows; }
    constexpr std::size_t cols() const { return Cols; }

private:
    std::array<T, Rows * Cols> data_{};
};

// Compile-time dimension check: R×K times K×C → R×C.
// Passing mismatched dimensions produces a deduction failure (no implicit cast).
template <typename T, std::size_t R, std::size_t K, std::size_t C>
Matrix<T, R, C> multiply(const Matrix<T, R, K>& a, const Matrix<T, K, C>& b) {
    Matrix<T, R, C> out;
    for (std::size_t r = 0; r < R; ++r) {
        for (std::size_t c = 0; c < C; ++c) {
            T sum{};
            for (std::size_t k = 0; k < K; ++k) sum += a(r, k) * b(k, c);
            out(r, c) = sum;
        }
    }
    return out;
}

template <typename T, std::size_t N>
Matrix<T, N, N> identity() {
    Matrix<T, N, N> out;
    for (std::size_t i = 0; i < N; ++i) out(i, i) = T{1};
    return out;
}

int main() {
    Matrix<double, 2, 3> a;
    Matrix<double, 3, 2> b;

    a(0, 0) = 1.0; a(0, 1) = 2.0; a(0, 2) = 3.0;
    a(1, 0) = 0.0; a(1, 1) = 0.0; a(1, 2) = 0.0;
    b(0, 0) = 4.0; b(0, 1) = 0.0;
    b(1, 0) = 5.0; b(1, 1) = 0.0;
    b(2, 0) = 6.0; b(2, 1) = 0.0;

    auto c = multiply(a, b);   // 2×3 × 3×2 → 2×2
    assert(c(0, 0) == 32.0);   // 1*4 + 2*5 + 3*6

    auto id = identity<int, 3>();
    assert(id(0, 0) == 1);
    assert(id(1, 1) == 1);
    assert(id(0, 1) == 0);

    // This should NOT compile — dimension mismatch.
    // Matrix<double, 3, 3> d;
    // auto bad = multiply(a, d);  // 2×3 × 3×3: K must match → 3==3 ✓ actually OK
    // auto bad2 = multiply(b, a); // 3×2 × 2×3 → 3×3 ✓
    // auto bad3 = multiply(a, a); // 2×3 × 2×3 → K=3≠2, deduction fails ✓

    std::cout << "ex02_hard passed\n";
}

