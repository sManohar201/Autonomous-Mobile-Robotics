#include <robot_math/vec3.hpp>
#include <cmath>

namespace robot_math {

double Vec3::magnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

Vec3 Vec3::normalized() const {
    double mag = magnitude();
    if (mag < 1e-12) return Vec3{0.0, 0.0, 0.0};
    return Vec3{x / mag, y / mag, z / mag};
}

double Vec3::dot(const Vec3& o) const {
    return x*o.x + y*o.y + z*o.z;
}

Vec3 Vec3::cross(const Vec3& o) const {
    return Vec3{
        y*o.z - z*o.y,
        z*o.x - x*o.z,
        x*o.y - y*o.x
    };
}

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(double s, const Vec3& v) {
    return Vec3{s * v.x, s * v.y, s * v.z};
}

Vec3 operator*(const Vec3& v, double s) {
    return s * v;
}

std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

} // namespace robot_math
