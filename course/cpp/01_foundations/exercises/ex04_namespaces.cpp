// Exercise 04 — Function Overloading and Namespaces
//
// TASK:
//   1. Implement the two existing magnitude overloads inside the geometry namespace.
//   2. Add a third overload: 3D magnitude taking three doubles.
//   3. Call all three overloads from main() and verify correct dispatch.
//   4. The commented-out line at the bottom is a namespace bug.
//      Move it to just after the #includes, observe what breaks, re-comment it,
//      and explain why in a comment.
//
// EXPECTED OUTPUT:
//   2D magnitude(3, 4) = 5
//   2D magnitude(1, 1) = 1.41421
//   3D magnitude(1, 2, 2) = 3

#include <iostream>
#include <cmath>

namespace geometry {

    // Overload 1: 2D magnitude from two doubles
    double magnitude(double x, double y) {
        // TODO
        return 0.0;
    }

    // Overload 2: 2D magnitude from two floats
    // The compiler picks this when arguments are float, not double.
    double magnitude(float x, float y) {
        // TODO: cast to double, then compute
        return 0.0;
    }

    // TODO: Overload 3 — 3D magnitude(double x, double y, double z)

} // namespace geometry

void demonstrate_local_using() {
    // Local using-declaration — safe, confined to this function only
    using geometry::magnitude;
    std::cout << "2D magnitude(1, 1) = " << magnitude(1.0f, 1.0f) << "\n";
}

int main() {
    std::cout << "2D magnitude(3, 4) = " << geometry::magnitude(3.0, 4.0) << "\n";

    demonstrate_local_using();

    // TODO: call the 3D overload here

    return 0;
}

// ── NAMESPACE BUG TASK ───────────────────────────────────────────────────────
// Move this line to just after the #includes at the top and observe what breaks.
// Then explain in a comment: why is file-scope 'using namespace' dangerous?
//
// using namespace geometry;
