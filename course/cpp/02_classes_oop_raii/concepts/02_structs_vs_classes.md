# 2.1 Structs vs Classes

The only technical difference between `struct` and `class` in C++ is the **default access level**: `struct` members are `public` by default; `class` members are `private` by default. Everything else — constructors, destructors, inheritance, virtual functions — works identically in both.

The conventional distinction is semantic:

| Use `struct` for | Use `class` for |
|---|---|
| Plain data aggregates (no invariants) | Types with invariants and behaviour |
| POD-like types passed between subsystems | Objects that manage resources |
| Simple message/record types | Encapsulated abstractions |

```cpp
#include <cstdint>
#include <cmath>

// struct: plain data, no invariants to protect.
// Anyone can read/write any field freely.
struct Point2D {
    double x;
    double y;
};

struct ImuReading {
    double accel_x, accel_y, accel_z;   // m/s^2
    double gyro_x,  gyro_y,  gyro_z;    // rad/s
    uint64_t timestamp_ns;
};

// class: has an invariant (radius must be > 0).
// The class protects that invariant through its interface.
class Circle {
public:
    // Constructor enforces the invariant.
    Circle(double cx, double cy, double radius)
        : cx_{cx}, cy_{cy}, radius_{radius}
    {
        if (radius_ <= 0.0) {
            throw std::invalid_argument("radius must be positive");
        }
    }

    double area()        const { return M_PI * radius_ * radius_; }
    double perimeter()   const { return 2.0 * M_PI * radius_; }
    double radius()      const { return radius_; }

    // Controlled mutation — invariant stays intact.
    void set_radius(double r) {
        if (r <= 0.0) throw std::invalid_argument("radius must be positive");
        radius_ = r;
    }

private:
    double cx_, cy_;
    double radius_;   // invariant: always > 0
};

int main() {
    Point2D p{1.0, 2.0};   // aggregate init, fine for plain data
    p.x = 3.0;              // direct access is the point

    Circle c{0.0, 0.0, 5.0};
    // c.radius_ = -1.0;    // compile error: private — invariant protected
    c.set_radius(6.0);      // goes through validation
}
```

### What Is an Invariant?

An **invariant** is a condition that must always be true about an object's internal state for the object to be considered valid. It's a rule you as the class designer define and then enforce through the constructor and every mutating method.

For `Circle`, the invariant is `radius_ > 0`. For a `Probability` class it might be `0.0 <= value_ <= 1.0`. For a robotics type:

```cpp
// Pose2d invariant: theta is always normalised to (-pi, pi].
// EKF filters rely on this — they never check it at runtime, they assume it.
class Pose2d {
public:
    Pose2d(double x, double y, double theta)
        : x_{x}, y_{y}, theta_{normalise(theta)}  // invariant established here
    {}

    void rotate(double delta) {
        theta_ = normalise(theta_ + delta);  // invariant maintained after every change
    }

    double theta() const { return theta_; }

private:
    static double normalise(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a <= -M_PI) a += 2.0 * M_PI;
        return a;
    }

    double x_, y_, theta_;
};
```

Using a `struct` here would let anyone write `pose.theta = 999.0` and silently break every downstream computation. The class protects against that.

**The rule of thumb:** if any combination of field values would produce a broken or meaningless object, you have an invariant and you need a class.

**Key takeaway:** Use `struct` when the type is just data and any combination of field values is valid. Use `class` when the type has invariants that must be maintained, or when it owns resources.
