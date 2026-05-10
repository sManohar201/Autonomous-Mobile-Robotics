#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <stdexcept>

template <typename T, std::size_t N>
class StaticVector {
public:
    constexpr StaticVector() = default;

    StaticVector(std::initializer_list<T> init) {
        if (init.size() != N) {
            throw std::invalid_argument("initializer size does not match vector dimension");
        }

        std::size_t i = 0;
        for (const T& value : init) {
            data_[i++] = value;
        }
    }

    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }

    constexpr std::size_t size() const { return N; }

    StaticVector& operator+=(const StaticVector& rhs) {
        for (std::size_t i = 0; i < N; ++i) {
            data_[i] += rhs.data_[i];
        }
        return *this;
    }

private:
    T data_[N]{};
};

template <typename T, std::size_t N>
StaticVector<T, N> operator+(StaticVector<T, N> lhs,
                             const StaticVector<T, N>& rhs) {
    lhs += rhs;
    return lhs;
}

template <typename T, std::size_t N>
T dot(const StaticVector<T, N>& a, const StaticVector<T, N>& b) {
    T sum{};
    for (std::size_t i = 0; i < N; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}

template <typename T, std::size_t N>
double norm(const StaticVector<T, N>& v) {
    return std::sqrt(static_cast<double>(dot(v, v)));
}

int main() {
    StaticVector<double, 3> a{1.0, 2.0, 3.0};
    StaticVector<double, 3> b{4.0, 5.0, 6.0};
    auto c = a + b;
    assert(c[0] == 5.0);
    assert(c[1] == 7.0);
    assert(c[2] == 9.0);
    assert(dot(a, b) == 32.0);
    assert(std::abs(norm(a) - std::sqrt(14.0)) < 1e-9);
    static_assert(StaticVector<int, 2>{}.size() == 2);

    std::cout << "ex02_static_vector_answer passed\n";
}

