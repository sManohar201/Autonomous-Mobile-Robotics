// Exercise 04 — Operator Overloading
//
// TASK:
//   Complete the Vec2 class by implementing every operator marked TODO.
//   Follow the canonical pattern: implement compound assignment (+=, -=, *=)
//   as members, then implement binary (+, -, *) as free functions using them.
//
// EXPECTED OUTPUT:
//   a = (1.0, 2.0)
//   b = (3.0, 4.0)
//   a + b = (4.0, 6.0)
//   a - b = (-2.0, -2.0)
//   2.0 * a = (2.0, 4.0)
//   a * 2.0 = (2.0, 4.0)
//   -a = (-1.0, -2.0)
//   a == a: 1
//   a == b: 0
//   a < b: 1
//   ++a: (2.0, 3.0)
//   a++: (2.0, 3.0)
//   a after post-increment: (3.0, 4.0)

#include <iostream>
#include <cmath>

class Vec2 {
public:
    double x, y;

    Vec2(double x = 0.0, double y = 0.0) : x(x), y(y) {}

    // TODO 1: operator+= (member)
    Vec2& operator+=(const Vec2& rhs);

    // TODO 2: operator-= (member)
    Vec2& operator-=(const Vec2& rhs);

    // TODO 3: scalar operator*= (member)
    Vec2& operator*=(double s);

    // TODO 4: unary operator- (member) — negate both components
    Vec2 operator-() const;

    // TODO 5: prefix operator++ — increment both components by 1, return *this
    Vec2& operator++();

    // TODO 6: postfix operator++ — save copy, increment, return saved copy
    //         Note: postfix takes a dummy int parameter
    Vec2 operator++(int);

    // TODO 7: operator== (member) — true if both x and y are equal
    bool operator==(const Vec2& rhs) const;

    // TODO 8: operator!= (member) — implement in terms of operator==
    bool operator!=(const Vec2& rhs) const;

    // TODO 9: operator< (member) — compare by magnitude (x²+y²)
    bool operator<(const Vec2& rhs) const;
};

// TODO 10: operator+ as a FREE function — implement using operator+=
Vec2 operator+(Vec2 lhs, const Vec2& rhs);

// TODO 11: operator- as a FREE function — implement using operator-=
Vec2 operator-(Vec2 lhs, const Vec2& rhs);

// TODO 12: scalar * Vec2 as a FREE function — implement using operator*=
Vec2 operator*(double s, Vec2 v);

// TODO 13: Vec2 * scalar as a FREE function — reuse scalar * Vec2
Vec2 operator*(Vec2 v, double s);

// TODO 14: operator<< (free function) — print in format "(x, y)"
std::ostream& operator<<(std::ostream& os, const Vec2& v);


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
