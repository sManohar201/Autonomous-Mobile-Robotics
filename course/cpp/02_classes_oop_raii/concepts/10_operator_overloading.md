# 2.9 Operator Overloading

C++ lets you give user-defined types the same syntactic operators as built-in types. This makes math-heavy robotics code (Eigen-style vectors, transforms, poses) read naturally.

### Member vs Free Function

| Operator | Preferred form | Reason |
|---|---|---|
| `=`, `[]`, `()`, `->` | Member only | Language requires it |
| `+=`, `-=`, `*=` | Member | Left operand is always this type |
| `+`, `-`, `*` | Free function | Allows conversion on left operand |
| `<<`, `>>` (stream) | Free function | Left operand is `ostream`, not your type |
| `==`, `!=`, `<` | Free function | Symmetric treatment of both operands |

```cpp
#include <iostream>
#include <cmath>
#include <stdexcept>

class Vec2 {
public:
    double x, y;

    Vec2(double x = 0.0, double y = 0.0) : x{x}, y{y} {}

    // --- Compound assignment (member) ---
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

    // --- Unary minus (member) ---
    Vec2 operator-() const {
        return Vec2{-x, -y};
    }

    // --- Prefix ++ (member) ---
    Vec2& operator++() {
        x += 1.0; y += 1.0;
        return *this;
    }

    // --- Postfix ++ (member): dummy int parameter distinguishes it ---
    Vec2 operator++(int) {
        Vec2 old{*this};   // save state
        ++(*this);         // delegate to prefix
        return old;        // return old state
    }

    double norm() const { return std::sqrt(x*x + y*y); }

    // --- Subscript (member, required) ---
    double& operator[](int i) {
        if (i == 0) return x;
        if (i == 1) return y;
        throw std::out_of_range("Vec2 index out of range");
    }
    double operator[](int i) const {
        if (i == 0) return x;
        if (i == 1) return y;
        throw std::out_of_range("Vec2 index out of range");
    }
};

// --- Binary arithmetic (free functions, defined in terms of +=) ---
Vec2 operator+(Vec2 lhs, const Vec2& rhs) { lhs += rhs; return lhs; }
Vec2 operator-(Vec2 lhs, const Vec2& rhs) { lhs -= rhs; return lhs; }
Vec2 operator*(Vec2 v,   double s)         { v  *= s;   return v;   }
Vec2 operator*(double s, Vec2 v)           { v  *= s;   return v;   }  // commutative

// --- Comparison (free functions) ---
bool operator==(const Vec2& a, const Vec2& b) {
    return a.x == b.x && a.y == b.y;
}
bool operator!=(const Vec2& a, const Vec2& b) { return !(a == b); }
bool operator< (const Vec2& a, const Vec2& b) {  // lexicographic
    return a.x < b.x || (a.x == b.x && a.y < b.y);
}

// --- Stream output (free function) ---
std::ostream& operator<<(std::ostream& os, const Vec2& v) {
    return os << "Vec2(" << v.x << ", " << v.y << ")";
}

// --- Stream input (free function) ---
std::istream& operator>>(std::istream& is, Vec2& v) {
    return is >> v.x >> v.y;
}

int main() {
    Vec2 a{1.0, 2.0};
    Vec2 b{3.0, 4.0};

    Vec2 c = a + b;
    std::cout << c << "\n";          // Vec2(4, 6)
    std::cout << -a << "\n";         // Vec2(-1, -2)
    std::cout << 2.0 * a << "\n";   // Vec2(2, 4)

    Vec2 d = a;
    d += b;
    std::cout << d << "\n";          // Vec2(4, 6)

    Vec2 e{1.0, 2.0};
    std::cout << std::boolalpha << (a == e) << "\n";   // true
    std::cout << (a < b) << "\n";                       // true

    Vec2 pre = a;
    ++pre;
    std::cout << pre << "\n";        // Vec2(2, 3)

    Vec2 post = a;
    Vec2 old = post++;
    std::cout << old << " -> " << post << "\n";  // Vec2(1,2) -> Vec2(2,3)

    // Don't overload: &&, ||, , (comma)
    // They lose short-circuit evaluation semantics when overloaded.
}
```

**Key takeaway:** Implement compound assignment operators (`+=`) as members, then implement binary operators (`+`) as free functions that call them. This avoids code duplication and enables implicit conversions on both operands.
