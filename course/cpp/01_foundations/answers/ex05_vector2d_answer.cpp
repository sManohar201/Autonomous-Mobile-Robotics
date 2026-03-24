// ANSWER — Exercise 05: vector2d.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Every member function definition outside the class body must be prefixed
// with Vector2d:: — this tells the compiler it is a member of that class.
// Without it:
//   1. 'const' qualifier is illegal on a free function
//   2. 'x' and 'y' are undeclared (they are member variables, not locals)
//   3. Linker reports "undefined reference" for every member function
// ─────────────────────────────────────────────────────────────────────────────

#include "vector2d.hpp"
#include <cmath>

namespace math {

// Constructor — initialiser list sets x and y before the body runs.
// More efficient than assignment in the body (no default-construct + assign).
Vector2d::Vector2d(double x, double y) : x(x), y(y) {}

// const member function — does not modify the object.
// Mark every read-only method const; the compiler enforces it.
double Vector2d::magnitude() const {
    return std::sqrt(x*x + y*y);
}

Vector2d Vector2d::normalized() const {
    double mag = magnitude();
    if (mag == 0.0) return Vector2d(0.0, 0.0);
    return Vector2d(x / mag, y / mag);
}

// dot product: a·b = ax*bx + ay*by
// Geometric meaning: |a||b|cos(θ). Zero = perpendicular vectors.
double Vector2d::dot(const Vector2d& other) const {
    return x*other.x + y*other.y;
}

// 2D cross product (z-component of 3D cross): ax*by - ay*bx
// Sign tells you which side b is on relative to a:
//   positive = b is counterclockwise from a
//   negative = b is clockwise from a
double Vector2d::cross(const Vector2d& other) const {
    return x*other.y - y*other.x;
}

// 2D rotation matrix applied to (x, y):
//   [cos θ  -sin θ] [x]   [x cos θ - y sin θ]
//   [sin θ   cos θ] [y] = [x sin θ + y cos θ]
Vector2d Vector2d::rotated(const double angle) const {
    return Vector2d(
        x * std::cos(angle) - y * std::sin(angle),
        x * std::sin(angle) + y * std::cos(angle)
    );
}

// angle_to — uses dot product formula: cos θ = (a·b) / (|a||b|)
//
// The clamp to [-1,1] before acos is critical.
// Floating-point rounding can produce values like 1.0000000000000002.
// acos of anything outside [-1,1] returns NaN — a silent bug that
// propagates through all downstream calculations.
double Vector2d::angle_to(const Vector2d& other) const {
    double cos_theta = dot(other) / (magnitude() * other.magnitude());
    cos_theta = cos_theta < -1.0 ? -1.0 : (cos_theta > 1.0 ? 1.0 : cos_theta);
    return std::acos(cos_theta);
}

// operator+ is a free function — it doesn't belong to either operand alone.
Vector2d operator+(const Vector2d& a, const Vector2d& b) {
    return Vector2d(a.x + b.x, a.y + b.y);
}

// Two overloads so both 2.0*v and v*2.0 work.
// The second reuses the first to avoid duplicating the formula.
Vector2d operator*(double scalar, const Vector2d& v) {
    return Vector2d(scalar * v.x, scalar * v.y);
}

Vector2d operator*(const Vector2d& v, double scalar) {
    return scalar * v;
}

// operator<< returns os& so calls can be chained:
//   std::cout << v1 << " and " << v2 << "\n";
std::ostream& operator<<(std::ostream& os, const Vector2d& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

} // namespace math
