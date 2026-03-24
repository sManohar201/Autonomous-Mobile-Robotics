// Exercise 05 — Multi-file CMake project
//
// EXPECTED OUTPUT:
//   v1 = (3, 4)
//   |v1| = 5
//   v1 normalised = (0.6, 0.8)
//   v1 . v2 = 11
//   v1 x v2 = 2
//   v1 + v2 = (4, 6)
//   2 * v1 = (6, 8)

#include <iostream>
#include "vector2d.hpp"

int main() {
    math::Vector2d v1(3.0, 4.0);
    math::Vector2d v2(1.0, 2.0);

    std::cout << "v1 = (" << v1.x << ", " << v1.y << ")\n";
    std::cout << "|v1| = "         << v1.magnitude()   << "\n";

    auto n = v1.normalized();
    std::cout << "v1 normalised = (" << n.x << ", " << n.y << ")\n";

    std::cout << "v1 . v2 = "  << v1.dot(v2)   << "\n";
    std::cout << "v1 x v2 = "  << v1.cross(v2) << "\n";

    auto sum = v1 + v2;
    std::cout << "v1 + v2 = (" << sum.x << ", " << sum.y << ")\n";

    auto scaled = 2.0 * v1;
    std::cout << "2 * v1 = (" << scaled.x << ", " << scaled.y << ")\n";

    return 0;
}
