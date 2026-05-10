// Exercise 02 - Class Templates and Non-Type Template Parameters
//
// Goal:
//   Implement a fixed-size vector whose dimension is part of the type.

#include <cassert>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <iostream>
#include <stdexcept>

template <typename T, std::size_t N>
class StaticVector {
public:
    // TODO 1: Default constructor should value-initialize all elements.

    // TODO 2: initializer_list constructor.
    // Throw std::invalid_argument if init.size() != N.

    // TODO 3: operator[] const and non-const.

    // TODO 4: constexpr size() returning N.

    // TODO 5: operator+=.

private:
    T data_[N]{};
};

// TODO 6: operator+ implemented in terms of operator+=.

// TODO 7: dot(a, b).

// TODO 8: norm(v), returning double.

int main() {
    // TODO: make these pass.
    // StaticVector<double, 3> a{1.0, 2.0, 3.0};
    // StaticVector<double, 3> b{4.0, 5.0, 6.0};
    // auto c = a + b;
    // assert(c[0] == 5.0);
    // assert(c[1] == 7.0);
    // assert(c[2] == 9.0);
    // assert(dot(a, b) == 32.0);
    // assert(std::abs(norm(a) - std::sqrt(14.0)) < 1e-9);
    // static_assert(StaticVector<int, 2>{}.size() == 2);

    std::cout << "ex02_static_vector passed\n";
}

