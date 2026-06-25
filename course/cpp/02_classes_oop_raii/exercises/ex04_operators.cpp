// Exercise 04 — Operator Overloading

#include <iostream>
#include <cmath>

class Vec2 {
public:
    double x, y;

    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    Vec2& operator+=(const Vec2& rhs) { x += rhs.x; y += rhs.y; return *this; }
    Vec2& operator-=(const Vec2& rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    Vec2& operator*=(double s)        { x *= s;     y *= s;     return *this; }

    Vec2 operator-() const { return Vec2(-x, -y); }

    Vec2& operator++()    { x += 1.0; y += 1.0; return *this; }
    Vec2  operator++(int) { Vec2 old = *this; ++(*this); return old; }

    bool operator==(const Vec2& rhs) const { return x == rhs.x && y == rhs.y; }
    bool operator!=(const Vec2& rhs) const { return !(*this == rhs); }

    bool operator<(const Vec2& rhs) const {
        return (x*x + y*y) < (rhs.x*rhs.x + rhs.y*rhs.y);
    }
};

Vec2 operator+(Vec2 lhs, const Vec2& rhs) { return lhs += rhs; }
Vec2 operator-(Vec2 lhs, const Vec2& rhs) { return lhs -= rhs; }
Vec2 operator*(double s, Vec2 v)          { return v *= s; }
Vec2 operator*(Vec2 v, double s)          { return s * v; }

std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

int main() {
    Vec2 a(1.0, 2.0);
    Vec2 b(3.0, 4.0);

    std::cout << "a = "       << a       << "\n";
    std::cout << "b = "       << b       << "\n";
    std::cout << "a + b = "   << a + b   << "\n";
    std::cout << "a - b = "   << a - b   << "\n";
    std::cout << "2.0 * a = " << 2.0 * a << "\n";
    std::cout << "a * 2.0 = " << a * 2.0 << "\n";
    std::cout << "-a = "      << -a      << "\n";

    std::cout << "a == a: " << (a == a) << "\n";
    std::cout << "a == b: " << (a == b) << "\n";
    std::cout << "a < b: "  << (a < b)  << "\n";

    std::cout << "++a: "    << ++a << "\n";
    std::cout << "a++: "    << a++ << "\n";
    std::cout << "a after post-increment: " << a << "\n";

    return 0;
}
