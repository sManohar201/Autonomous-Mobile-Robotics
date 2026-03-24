// ANSWER — Exercise 04: Function Overloading and Namespaces
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <cmath>

namespace geometry {

    // Overload 1: exact match for two doubles.
    double magnitude(double x, double y) {
        return std::sqrt(x*x + y*y);
    }

    // Overload 2: exact match for two floats.
    // Without this, magnitude(1.0f, 1.0f) would implicitly convert float→double
    // and call overload 1. The compiler always prefers exact match over conversion.
    // In performance-critical code, preventing implicit conversions avoids subtle
    // precision bugs and can be faster.
    double magnitude(float x, float y) {
        return std::sqrt(static_cast<double>(x)*static_cast<double>(x) +
                         static_cast<double>(y)*static_cast<double>(y));
    }

    // Overload 3: three doubles — distinct from overload 1 by argument count.
    double magnitude(double x, double y, double z) {
        return std::sqrt(x*x + y*y + z*z);
    }

} // namespace geometry

// ── Namespace usage bug explanation ──────────────────────────────────────────
// 'using namespace geometry;' at file scope imports every name in geometry
// into the global namespace for this entire translation unit.
// In a larger codebase:
//
//   #include "geometry.hpp"     // defines geometry::magnitude
//   #include "signal_utils.hpp" // also defines magnitude() at global scope
//   using namespace geometry;   // now two 'magnitude' symbols are visible
//   magnitude(1.0, 2.0);        // ambiguous — compiler error or wrong function
//
// In a header file it is even worse: every file that includes that header
// gets the using namespace silently applied.
//
// Safe alternatives:
//   1. geometry::magnitude(...)            — fully qualified, always unambiguous
//   2. using geometry::magnitude; inside a function — scoped, imports one name
// ─────────────────────────────────────────────────────────────────────────────

void demonstrate_local_using() {
    using geometry::magnitude;   // scoped to this function only
    std::cout << "2D magnitude(1, 1) = " << magnitude(1.0f, 1.0f) << "\n";
}

int main() {
    std::cout << "2D magnitude(3, 4) = " << geometry::magnitude(3.0, 4.0) << "\n";
    demonstrate_local_using();
    std::cout << "3D magnitude(1, 2, 2) = " << geometry::magnitude(1.0, 2.0, 2.0) << "\n";
    return 0;
}
