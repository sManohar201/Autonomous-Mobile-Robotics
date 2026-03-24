// vector2d.cpp — implementation of the math::Vector2d library
//
// TASK: implement every function declared in vector2d.hpp.
// The main.cpp will not link until all definitions are present.

#include "vector2d.hpp"
#include <cmath>

namespace math {

Vector2d::Vector2d(double x, double y) : x(x), y(y) {}

double Vector2d::magnitude() const {
    // TODO
    return 0.0;
}

Vector2d Vector2d::normalized() const {
    // TODO
    return Vector2d(0.0, 0.0);
}

double Vector2d::dot(const Vector2d& other) const {
    // TODO
    return 0.0;
}

double Vector2d::cross(const Vector2d& other) const {
    // TODO: formula is ax*by - ay*bx
    return 0.0;
}

Vector2d Vector2d::rotated(const double angle) const {
    // TODO: apply 2D rotation matrix
    // new_x = x*cos(angle) - y*sin(angle)
    // new_y = x*sin(angle) + y*cos(angle)
    return Vector2d(0.0, 0.0);
}

double Vector2d::angle_to(const Vector2d& other) const {
    // TODO: dot product formula — clamp result to [-1,1] before acos
    return 0.0;
}

Vector2d operator+(const Vector2d& a, const Vector2d& b) {
    // TODO
    return Vector2d(0.0, 0.0);
}

Vector2d operator*(double scalar, const Vector2d& v) {
    // TODO
    return Vector2d(0.0, 0.0);
}

Vector2d operator*(const Vector2d& v, double scalar) {
    return scalar * v;
}

std::ostream& operator<<(std::ostream& os, const Vector2d& v) {
    // TODO: print in format (x, y)
    return os;
}

} // namespace math
