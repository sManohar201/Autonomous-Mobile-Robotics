# 2.2 Constructors & Destructors

A **constructor** runs when an object is created; a **destructor** runs when it goes out of scope (or is deleted). Stack objects are destroyed in **LIFO order** — last constructed, first destroyed.

### Default and Parameterized Constructors

```cpp
#include <iostream>
#include <string>

class Sensor {
public:
    // Default constructor — compiler generates one only if you define none.
    // Once you write any constructor, the default is suppressed unless you
    // explicitly request it.
    Sensor() : name_{"unnamed"}, rate_hz_{10.0} {
        std::cout << "Sensor default ctor: " << name_ << "\n";
    }

    // Parameterized constructor.
    Sensor(std::string name, double rate_hz)
        : name_{std::move(name)}, rate_hz_{rate_hz}
    {
        std::cout << "Sensor ctor: " << name_ << " @ " << rate_hz_ << " Hz\n";
    }

    // Destructor — runs automatically when object lifetime ends.
    ~Sensor() {
        std::cout << "Sensor dtor: " << name_ << "\n";
    }

    const std::string& name() const { return name_; }

private:
    std::string name_;
    double rate_hz_;
};

// Constructor overloading — same name, different signatures.
class Vector3 {
public:
    Vector3() : x_{0.0}, y_{0.0}, z_{0.0} {}
    Vector3(double val) : x_{val}, y_{val}, z_{val} {}       // fill constructor
    Vector3(double x, double y, double z) : x_{x}, y_{y}, z_{z} {}

    void print() const {
        std::cout << "(" << x_ << ", " << y_ << ", " << z_ << ")\n";
    }

private:
    double x_, y_, z_;
};

int main() {
    {
        Sensor lidar{"Velodyne", 20.0};
        Sensor camera{"ZED", 30.0};
        // camera goes out of scope first (LIFO), then lidar.
    }   // prints: dtor ZED, then dtor Velodyne

    Vector3 zero;            // (0, 0, 0)
    Vector3 ones{1.0};       // (1, 1, 1)
    Vector3 v{1.0, 2.0, 3.0};

    zero.print();
    ones.print();
    v.print();
}
```

### The `explicit` Keyword

Without `explicit`, a single-argument constructor doubles as an **implicit conversion operator**. The compiler is allowed to call it automatically whenever it needs to make types match — silently, without telling you.

```cpp
// WITHOUT explicit — dangerous
class Angle {
public:
    Angle(double radians) : radians_{radians} {}   // implicit conversion allowed
    double radians() const { return radians_; }
private:
    double radians_;
};

void rotate(Angle a) { std::cout << "rotating by " << a.radians() << " rad\n"; }

rotate(1.57);    // compiles silently — compiler secretly calls Angle(1.57)
rotate(90.0);    // also compiles — but is this degrees or radians? Silent bug.
```

The compiler sees `rotate` wants an `Angle`, you passed a `double`, so it quietly constructs one. No warning, no error — just a bug if you accidentally passed degrees instead of radians.

With `explicit`, you must be intentional at every call site:

```cpp
// WITH explicit — safe
class Angle {
public:
    explicit Angle(double radians) : radians_{radians} {}
    double radians() const { return radians_; }
private:
    double radians_;
};

rotate(1.57);           // COMPILE ERROR — caught immediately
rotate(Angle{1.57});    // OK — caller explicitly says "this is an Angle in radians"
rotate(Angle(1.57));    // also OK
```

The compile error forces the caller to think: "am I passing radians or degrees?" That is exactly what you want.

**Other types where silent conversion causes real bugs:**

```cpp
Buffer(int size);         // Buffer b = 5;    — was 5 the size or a fill value?
Probability(double p);    // func(1.5);       — 1.5 is not a valid probability
Seconds(double t);        // timer.wait(60);  — minutes? milliseconds?
Metres(double d);         // move(100);       — centimetres? feet?
```

**The rule:** mark `explicit` on any single-argument constructor unless you genuinely want implicit conversion (rare — `std::string` from `const char*` is one of the few good examples).

### Delegating Constructors

A constructor can call another constructor in the same class to avoid duplicating logic.

```cpp
class Rectangle {
public:
    // Primary constructor — all logic lives here.
    Rectangle(double w, double h, std::string label)
        : width_{w}, height_{h}, label_{std::move(label)}
    {
        if (width_ <= 0 || height_ <= 0)
            throw std::invalid_argument("dimensions must be positive");
    }

    // Delegating constructor — delegates to the primary.
    Rectangle(double w, double h)
        : Rectangle{w, h, "unnamed"}   // delegation
    {}

    // Another delegate for a square.
    explicit Rectangle(double side)
        : Rectangle{side, side, "square"}
    {}

    double area() const { return width_ * height_; }

private:
    double width_, height_;
    std::string label_;
};

int main() {
    Rectangle r1{3.0, 4.0, "room"};
    Rectangle r2{3.0, 4.0};
    Rectangle sq{5.0};

    std::cout << r1.area() << "\n";  // 12
    std::cout << sq.area() << "\n";  // 25
}
```

**Key takeaway:** Always mark single-argument constructors `explicit` unless you intentionally want an implicit conversion. Use delegating constructors to centralise validation logic.
