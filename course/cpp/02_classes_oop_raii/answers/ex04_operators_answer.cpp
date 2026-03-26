// Exercise 04 — Operator Overloading
// ANSWER FILE

#include <iostream>
#include <iomanip>
#include <cmath>

class Vec2 {
public:
    double x, y;

    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    // Compound assignment operators (members) ─────────────────────────────────

    Vec2& operator+=(const Vec2& rhs) {
        x += rhs.x;
        y += rhs.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& rhs) {
        x -= rhs.x;
        y -= rhs.y;
        return *this;
    }

    Vec2& operator*=(double s) {
        x *= s;
        y *= s;
        return *this;
    }

    // Unary negation ──────────────────────────────────────────────────────────

    Vec2 operator-() const {
        return Vec2(-x, -y);
    }

    // Increment operators ─────────────────────────────────────────────────────

    // Prefix: increment both components, return *this
    Vec2& operator++() {
        x += 1.0;
        y += 1.0;
        return *this;
    }

    // Postfix: save copy, increment, return saved copy
    Vec2 operator++(int) {
        Vec2 old(*this);
        ++(*this);
        return old;
    }

    // Comparison operators ────────────────────────────────────────────────────

    bool operator==(const Vec2& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    bool operator!=(const Vec2& rhs) const {
        return !(*this == rhs);
    }

    // Less-than: compare by magnitude (x² + y²)
    bool operator<(const Vec2& rhs) const {
        return (x * x + y * y) < (rhs.x * rhs.x + rhs.y * rhs.y);
    }
};

// Binary arithmetic — free functions using compound assignment ─────────────

Vec2 operator+(Vec2 lhs, const Vec2& rhs) {
    lhs += rhs;
    return lhs;
}

Vec2 operator-(Vec2 lhs, const Vec2& rhs) {
    lhs -= rhs;
    return lhs;
}

// scalar * Vec2
Vec2 operator*(double s, Vec2 v) {
    v *= s;
    return v;
}

// Vec2 * scalar — reuse scalar * Vec2
Vec2 operator*(Vec2 v, double s) {
    return s * v;
}

// Stream output ───────────────────────────────────────────────────────────────
std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    os << std::fixed << std::setprecision(1)
       << "(" << v.x << ", " << v.y << ")";
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
