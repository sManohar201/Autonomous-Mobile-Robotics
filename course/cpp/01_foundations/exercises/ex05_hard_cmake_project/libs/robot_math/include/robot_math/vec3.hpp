#pragma once
#include <ostream>

namespace robot_math {

struct Vec3 {
    double x, y, z;

    constexpr Vec3(double x = 0.0, double y = 0.0, double z = 0.0)
        : x(x), y(y), z(z) {}

    double magnitude()    const;
    Vec3   normalized()   const;
    double dot(const Vec3& o)   const;
    Vec3   cross(const Vec3& o) const;
};

Vec3 operator+(const Vec3& a, const Vec3& b);
Vec3 operator-(const Vec3& a, const Vec3& b);
Vec3 operator*(double s,       const Vec3& v);
Vec3 operator*(const Vec3& v,  double s);
std::ostream& operator<<(std::ostream& os, const Vec3& v);

} // namespace robot_math
