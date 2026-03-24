#pragma once
#include <iostream>

// vector2d.hpp — 2D vector type
// Public interface only. Definitions live in src/vector2d.cpp.

namespace math {

struct Vector2d {
    double x;
    double y;

    Vector2d(double x, double y);

    double   magnitude() const;
    Vector2d normalized() const;
    double   dot(const Vector2d& other) const;
    double   cross(const Vector2d& other) const;     // ax*by - ay*bx
    Vector2d rotated(const double angle) const;
    double   angle_to(const Vector2d& other) const;
};

Vector2d operator+(const Vector2d& a, const Vector2d& b);
Vector2d operator*(double scalar, const Vector2d& v);
Vector2d operator*(const Vector2d& v, double scalar);
std::ostream& operator<<(std::ostream& os, const Vector2d& v);

} // namespace math
