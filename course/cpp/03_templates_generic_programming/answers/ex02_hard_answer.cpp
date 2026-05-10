#include <array>
#include <cassert>
#include <cstddef>
#include <iostream>

template <typename T, std::size_t Rows, std::size_t Cols>
class Matrix {
public:
    T& operator()(std::size_t r, std::size_t c) { return data_[r * Cols + c]; }
    const T& operator()(std::size_t r, std::size_t c) const { return data_[r * Cols + c]; }

private:
    std::array<T, Rows * Cols> data_{};
};

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
    b(0, 0) = 4.0; b(1, 0) = 5.0; b(2, 0) = 6.0;
    auto c = multiply(a, b);
    assert(c(0, 0) == 32.0);
    auto id = identity<int, 3>();
    assert(id(2, 2) == 1);
    std::cout << "ex02_hard_answer passed\n";
}

