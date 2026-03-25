# Module 2 — Classes, OOP & RAII

**Prerequisites:** Module 1 (compilation pipeline, types, const, pointers/refs, namespaces, CMake)
**Goal:** Understand C++ object-oriented mechanics and the resource-management idioms used throughout production robotics code (ROS2 nodes, Eigen types, sensor drivers).

---

## 2.1 Structs vs Classes

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

---

## 2.2 Constructors & Destructors

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

---

## 2.3 Member Initializer Lists

The **member initializer list** (MIL) runs before the constructor body. It is not just a style choice — for certain members it is the only option.

### Why MIL Is Preferred

```cpp
#include <string>
#include <iostream>

class Node {
public:
    // WRONG: const member cannot be assigned in the body.
    // Uncommenting the body-assignment version gives a compile error.
    //
    // Node(int id) {
    //     id_ = id;     // error: assignment to const member
    // }

    // RIGHT: const and reference members MUST be in the MIL.
    Node(int id, std::string name, double& shared_val)
        : id_{id}                    // const int — only place to set it
        , name_{std::move(name)}     // string: MIL calls move-ctor directly
        , ref_{shared_val}           // reference — must bind here
    {}

    void print() const {
        std::cout << "Node " << id_ << ": " << name_ << "\n";
    }

private:
    const int id_;          // MUST be in MIL
    std::string name_;      // prefer MIL (avoids default-then-copy)
    double& ref_;           // MUST be in MIL
};
```

### MIL Order Matches Declaration Order

The MIL initializes members in **declaration order**, not the order they appear in the MIL. Writing them out of order compiles but is misleading.

```cpp
// Safe: MIL order matches declaration order.
class Pose {
public:
    Pose(double x, double y, double yaw)
        : x_{x}, y_{y}, yaw_{yaw}   // same order as declared below — fine
    {}
private:
    double x_, y_, yaw_;
};

// DANGER: MIL order disagrees with declaration order.
// Members are always initialized in DECLARATION order, not MIL order.
// Here: sum_ is declared first, so it is initialized first —
// but its MIL entry uses x_ and y_, which haven't been initialized yet.
// Result: sum_ gets garbage. Compiler warns with -Wreorder.
class Stats {
public:
    Stats(double x, double y)
        : x_{x}, y_{y}, sum_{x_ + y_}   // BUG: sum_ runs before x_ and y_
    {}
    double sum() const { return sum_; }
private:
    double sum_;   // declared first → initialized first, even though last in MIL
    double x_;
    double y_;
};

// Fix: declare members in dependency order.
class StatsSafe {
public:
    StatsSafe(double x, double y)
        : x_{x}, y_{y}, sum_{x_ + y_}   // fine: x_ and y_ declared before sum_
    {}
private:
    double x_;
    double y_;
    double sum_;   // declared last → initialized last, x_ and y_ are ready
};
```

### Body Assignment Silently Failing

```cpp
// This does NOT compile — shown to illustrate the category of error.
//
// class BadNode {
// public:
//     BadNode(int id) {
//         id_ = id;    // error: cannot assign to const member 'id_'
//     }
// private:
//     const int id_;
// };
//
// The const member is default-initialized (undefined for int) before the
// body runs. By the time the body assignment executes, it's too late —
// the object is already "constructed" with an indeterminate id_.
```

**Key takeaway:** Always initialize members in the MIL, in declaration order. `const` and reference members have no alternative — the compiler enforces it.

---

## 2.4 `this` Pointer & const Member Functions

Inside a non-static member function, `this` is a pointer to the object the function was called on. Its type is `ClassName*` (or `const ClassName*` in a const member function).

### When You Need `this` Explicitly

```cpp
#include <iostream>

class Filter {
public:
    Filter(double cutoff, double gain)
        : cutoff_{cutoff}, gain_{gain}
    {}

    // Name shadowing: parameter shadows member — use this-> to disambiguate.
    void set_cutoff(double cutoff) {
        this->cutoff_ = cutoff;   // this-> resolves ambiguity
    }

    // Better style: use trailing underscore for members (avoids the problem).
    void set_gain(double gain) {
        gain_ = gain;   // no ambiguity: gain vs gain_
    }

    // Returning *this enables method chaining — call multiple methods in one line.
    // Return type must be Filter& (reference), not Filter (value).
    // Returning by value would return a copy — the chain would operate on
    // throwaway temporaries and the original object would never change.
    Filter& set(double cutoff, double gain) {
        cutoff_ = cutoff;
        gain_   = gain;
        return *this;   // hand the same object back so the next call has something to work on
    }

    // const member function: compiler ensures this function
    // cannot modify any non-mutable members.
    double output(double input) const {
        return input * gain_;   // OK: reading
        // gain_ = 2.0;        // compile error in a const function
    }

    void print() const {
        std::cout << "cutoff=" << cutoff_ << " gain=" << gain_ << "\n";
    }

private:
    double cutoff_;
    double gain_;
};

// A function accepting a const reference can only call const member functions.
void inspect(const Filter& f) {
    f.print();          // OK: print() is const
    // f.set_gain(1.0); // compile error: set_gain() is not const
}

int main() {
    Filter f{100.0, 0.5};

    // Method chaining via returning *this.
    f.set(200.0, 0.8).set(150.0, 0.6).print();

    inspect(f);
}
```

### `this` vs `*this` — and Why Reference, Not Pointer

`this` is a pointer (`Filter*`). `*this` dereferences it to get the object itself (`Filter`). When you return `*this` with return type `Filter&`, you're binding a reference to the object — which points back at the same memory as `this`.

```cpp
// What this means step by step:
f.set(200.0, 0.8).set(150.0, 0.6).print();
//  f.set(200.0, 0.8)  → modifies f, returns Filter& referring to f
//           .set(150.0, 0.6)  → called on f (same object), returns Filter& to f
//                    .print() → called on f
```

You could technically return `this` (a pointer) and chain with `->` instead:

```cpp
Filter* set(double cutoff, double gain) {
    cutoff_ = cutoff; gain_ = gain;
    return this;
}
f.set(200.0, 0.8)->set(150.0, 0.6)->print();   // works, but ugly
```

| Return type | Chain syntax | Signals to reader |
|---|---|---|
| `Filter&` (`*this`) | `f.a().b().c()` | same object, always valid |
| `Filter*` (`this`) | `f.a()->b()->c()` | might be null? |

References communicate "this is the same object, it always exists." Pointers communicate "this might be null, check before using." Method chaining is always the former, so `Filter&` is the right choice.

**Key takeaway:** Mark every member function that does not modify object state as `const`. This allows the function to be called on `const` objects and through `const` references, which is the norm in robotics APIs (sensor callbacks, read-only inspection).

---

## 2.5 Copy Semantics

When you copy an object, C++ calls the **copy constructor** (initialization from another object) or **copy assignment operator** (assignment after both objects exist).

### Shallow Copy and the Double-Free Bug

If your class owns heap memory through a raw pointer, the compiler-generated copy constructor copies the pointer value — both objects then point to the same allocation. Destroying either one leaves the other with a dangling pointer; destroying both is undefined behaviour (double-free).

```cpp
#include <cstring>
#include <iostream>
#include <stdexcept>

// Dangerous: raw pointer without custom copy semantics.
class BadBuffer {
public:
    explicit BadBuffer(std::size_t size)
        : size_{size}, data_{new double[size]}
    {
        std::fill(data_, data_ + size_, 0.0);
    }

    ~BadBuffer() {
        delete[] data_;   // problem: if data_ is shared, this is a double-free
    }

    // No copy constructor or copy assignment defined.
    // The compiler generates:
    //   BadBuffer(const BadBuffer& o) : size_{o.size_}, data_{o.data_} {}
    // This is a SHALLOW copy: both objects share the same data_ pointer.

    double* data() { return data_; }

private:
    std::size_t size_;
    double* data_;
};

// void demonstrate_bug() {
//     BadBuffer a{4};
//     BadBuffer b{a};   // shallow copy: b.data_ == a.data_
//     // When b is destroyed: delete[] b.data_  -> fine
//     // When a is destroyed: delete[] a.data_  -> DOUBLE FREE: undefined behaviour
// }
```

### Rule of Three: Deep Copy Done Right

If you define any of {**destructor**, **copy constructor**, **copy assignment operator**}, you almost certainly need all three.

```cpp
class Buffer {
public:
    explicit Buffer(std::size_t size)
        : size_{size}, data_{new double[size]}
    {
        std::fill(data_, data_ + size_, 0.0);
        std::cout << "Buffer ctor, size=" << size_ << "\n";
    }

    // Destructor (1 of 3)
    ~Buffer() {
        std::cout << "Buffer dtor, size=" << size_ << "\n";
        delete[] data_;
    }

    // Copy constructor (2 of 3) — deep copy
    Buffer(const Buffer& other)
        : size_{other.size_}, data_{new double[other.size_]}
    {
        std::copy(other.data_, other.data_ + other.size_, data_);
        std::cout << "Buffer copy ctor\n";
    }

    // Copy assignment operator (3 of 3) — deep copy
    Buffer& operator=(const Buffer& other) {
        if (this == &other) return *this;   // self-assignment guard

        // Allocate new memory before releasing old (strong exception safety).
        double* tmp = new double[other.size_];
        std::copy(other.data_, other.data_ + other.size_, tmp);

        delete[] data_;      // release old resource
        data_ = tmp;
        size_ = other.size_;

        std::cout << "Buffer copy assign\n";
        return *this;
    }

    std::size_t size() const { return size_; }
    double& operator[](std::size_t i) { return data_[i]; }
    double  operator[](std::size_t i) const { return data_[i]; }

private:
    std::size_t size_;
    double* data_;
};

int main() {
    Buffer a{4};
    a[0] = 1.0; a[1] = 2.0;

    // ── Copy constructor ─────────────────────────────────────────────────────
    // Called when a NEW object is being initialized from an existing one.
    Buffer b{a};       // same as: Buffer b = a;
    b[0] = 99.0;

    std::cout << "a[0]=" << a[0] << "  b[0]=" << b[0] << "\n";
    // a[0]=1   b[0]=99  — independent allocations, no aliasing

    // ── Copy assignment operator ─────────────────────────────────────────────
    // Called when BOTH objects already exist and you assign one to the other.
    Buffer c{4};       // c already exists — has its own allocation
    c[0] = 7.0;

    c = a;             // copy assignment: c releases its old memory,
                       // allocates new memory, copies a's data into it
    c[0] = 55.0;

    std::cout << "a[0]=" << a[0] << "  c[0]=" << c[0] << "\n";
    // a[0]=1   c[0]=55  — still independent

    // ── Self-assignment (must be safe) ───────────────────────────────────────
    a = a;             // the self-assignment guard handles this — no crash
    std::cout << "a[0] after self-assign=" << a[0] << "\n";
    // a[0]=1  — unchanged
}
```

### Self-Assignment Guard — When Is It Actually Required?

You rarely write `a = a` deliberately. But it happens **indirectly**:

```cpp
buffers[i] = buffers[j];         // what if i == j at runtime?

void update(Buffer& dst, const Buffer& src) { dst = src; }
update(buffers[idx], buffers[idx]);   // same object for both arguments
```

Whether the guard is **required** depends on which pattern you use:

**Pattern 1 — delete first, then allocate (guard required):**
```cpp
Buffer& operator=(const Buffer& other) {
    delete[] data_;                                   // 1. free our memory
    size_ = other.size_;
    data_ = new double[other.size_];                  // 2. allocate
    std::copy(other.data_, ..., data_);               // 3. copy from other.data_
    return *this;
    // In self-assignment: step 1 frees data_ which IS other.data_.
    // Step 3 copies from freed memory — undefined behaviour.
    // The guard is REQUIRED here.
}
```

**Pattern 2 — allocate tmp first, then delete (guard not required, but kept as optimisation):**
```cpp
Buffer& operator=(const Buffer& other) {
    double* tmp = new double[other.size_];            // 1. allocate new block
    std::copy(other.data_, ..., tmp);                 // 2. copy into tmp — other.data_ still alive
    delete[] data_;                                   // 3. free old block
    data_ = tmp;
    size_ = other.size_;
    return *this;
    // In self-assignment: tmp is filled from other.data_ BEFORE we delete anything.
    // Correctness is preserved even without the guard.
    // Guard is kept only as a cheap short-circuit (skip unnecessary work).
}
```

The `Buffer` example above uses Pattern 2, so the guard is a performance optimisation, not a correctness requirement. Pattern 1 — which you'll see in simpler implementations — needs it for safety.

**Key takeaway:** Any class owning a raw pointer must implement the Rule of Three. In practice, prefer `std::vector` or a smart pointer, which implement these correctly for you.

---

## 2.6 Move Semantics

**Move semantics** let you transfer ownership of resources instead of copying them — critical for performance when returning large objects or inserting into containers.

### lvalue vs rvalue Intuition

An **lvalue** has a name and a persistent address. An **rvalue** is a temporary with no name — it will cease to exist after the expression. The rule of thumb: *if you can take its address, it's an lvalue.*

```cpp
int x = 5;         // x is lvalue, 5 is rvalue
std::string s = std::string{"hello"};
//           ^^ temporary rvalue
```

### Rvalue References and Move Operations

```cpp
#include <iostream>
#include <cstring>
#include <utility>  // std::move

class PointCloud {
public:
    explicit PointCloud(std::size_t n)
        : size_{n}, points_{new float[n * 3]}
    {
        std::cout << "PointCloud ctor n=" << n << "\n";
    }

    ~PointCloud() {
        std::cout << "PointCloud dtor (points_=" << (void*)points_ << ")\n";
        delete[] points_;
    }

    // Copy constructor — expensive: allocates and copies all points.
    PointCloud(const PointCloud& o)
        : size_{o.size_}, points_{new float[o.size_ * 3]}
    {
        std::copy(o.points_, o.points_ + o.size_ * 3, points_);
        std::cout << "PointCloud COPY ctor\n";
    }

    // Copy assignment.
    PointCloud& operator=(const PointCloud& o) {
        if (this == &o) return *this;
        float* tmp = new float[o.size_ * 3];
        std::copy(o.points_, o.points_ + o.size_ * 3, tmp);
        delete[] points_;
        points_ = tmp;
        size_   = o.size_;
        std::cout << "PointCloud COPY assign\n";
        return *this;
    }

    // Move constructor — cheap: steal the pointer, null out source.
    // Takes an rvalue reference (&&).
    PointCloud(PointCloud&& o) noexcept
        : size_{o.size_}, points_{o.points_}
    {
        o.points_ = nullptr;  // source no longer owns the memory
        o.size_   = 0;
        std::cout << "PointCloud MOVE ctor\n";
    }

    // Move assignment.
    PointCloud& operator=(PointCloud&& o) noexcept {
        if (this == &o) return *this;
        delete[] points_;     // release our current resource
        points_   = o.points_;
        size_     = o.size_;
        o.points_ = nullptr;
        o.size_   = 0;
        std::cout << "PointCloud MOVE assign\n";
        return *this;
    }

    std::size_t size() const { return size_; }

private:
    std::size_t size_;
    float* points_;
};

PointCloud make_cloud(std::size_t n) {
    PointCloud pc{n};
    // ... fill points ...
    return pc;   // NRVO may elide the copy/move entirely
}

int main() {
    PointCloud a{1000};

    // std::move is a cast to rvalue — it doesn't move anything by itself.
    // The move constructor is what actually transfers ownership.
    PointCloud b{std::move(a)};   // move ctor called; a is now empty
    std::cout << "a.size=" << a.size() << " b.size=" << b.size() << "\n";
    // a.size=0  b.size=1000

    PointCloud c{500};
    c = std::move(b);             // move assign; b is now empty

    // Function returning a local object — typically move-elided (NRVO).
    PointCloud d = make_cloud(200);
}
```

### Rule of Five

The Rule of Three extends to the **Rule of Five** when move operations are relevant:

1. Destructor
2. Copy constructor
3. Copy assignment operator
4. Move constructor
5. Move assignment operator

The compiler generates move operations automatically only when you have **not** declared any of {destructor, copy constructor, copy assignment}. As soon as you declare any one of them (as you must for raw-pointer classes), you must explicitly define the move operations too — or `= default` / `= delete` them.

```cpp
class ManagedResource {
public:
    explicit ManagedResource(int id) : id_{id} {}

    ~ManagedResource()                                        = default; // 1
    ManagedResource(const ManagedResource&)                   = default; // 2
    ManagedResource& operator=(const ManagedResource&)        = default; // 3
    ManagedResource(ManagedResource&&) noexcept               = default; // 4
    ManagedResource& operator=(ManagedResource&&) noexcept    = default; // 5

private:
    int id_;
};
```

### Why `std::move` Only Helps Heap-Owning Types

`std::move` is just a cast to an rvalue reference — it doesn't move anything by itself. It tells the compiler "treat this as a temporary you can steal from." Whether that's actually faster depends entirely on what the type owns.

For a `double`, there's nothing to steal — it's 8 bytes sitting directly in the object. Copy and move are identical:

```cpp
double a = 3.14;
double b = std::move(a);   // identical to: double b = a;
// a is still 3.14 — nothing was moved, bytes were just copied
```

For `std::string`, there's a heap allocation to steal:

```cpp
std::string a = "a very long string that lives on the heap...";
//  a:  [ ptr ──→ heap buffer | size | capacity ]

std::string b = std::move(a);
//  b:  [ ptr ──→ heap buffer | size | capacity ]  (same heap block, no copy)
//  a:  [ ptr=null | size=0 | capacity=0 ]          (left empty)
```

Move just transfers the pointer. No heap allocation, no character copy.

| Type | Owns heap memory? | Move faster than copy? |
|---|---|---|
| `double`, `int`, `float` | no | no — same cost |
| `std::string` | yes (usually) | yes |
| `std::vector<T>` | yes | yes |
| `std::unique_ptr` | yes (exclusive) | yes |
| raw pointer `T*` | no (just an address) | no |
| `PointCloud` (above) | yes (`points_` on heap) | yes |

In practice: when constructing a class, `std::move` string and vector members in the constructor, don't bother for numeric types:

```cpp
class Sensor {
public:
    Sensor(std::string name, double rate_hz)
        : name_{std::move(name)}   // moves the heap buffer — one allocation total
        , rate_hz_{rate_hz}        // double: move = copy, no difference
    {}
private:
    std::string name_;
    double rate_hz_;
};
```

### Why `noexcept` on Move Operations

`noexcept` on move operations is not about correctness — it's about enabling an optimisation in the standard library, specifically `std::vector`.

When a `std::vector` runs out of capacity it allocates a bigger buffer and transfers all existing elements. It has a safety rule: if anything goes wrong during the transfer, the original vector must be left unchanged.

- With **copy**: if a copy throws midway, the originals are untouched — easy to roll back.
- With **move**: if a move throws midway, the original elements have already been gutted — nothing to roll back.

So `std::vector` asks: *can this move throw?*

```cpp
// WITHOUT noexcept on PointCloud's move ctor:
//   vector falls back to COPYING every PointCloud during reallocation
//   → expensive heap allocations for every element

// WITH noexcept on PointCloud's move ctor:
//   vector MOVES every PointCloud — just pointer swaps, near zero cost
std::vector<PointCloud> clouds;
clouds.push_back(PointCloud{1000});   // triggers reallocation
```

A well-written move only swaps pointers and integers — it genuinely cannot throw. `noexcept` is your promise to the compiler, and in return the standard library trusts you enough to use moves:

```cpp
PointCloud(PointCloud&& o) noexcept   // promised: no throw
    : size_{o.size_}, points_{o.points_}
{
    o.points_ = nullptr;
    o.size_   = 0;
    // no allocations, no exceptions possible — promise is valid
}
```

If you break the promise and it does throw, `std::terminate()` is called immediately.

### NRVO — Named Return Value Optimisation

When you return a **named local variable** by value, the compiler is allowed to construct it directly in the caller's memory — eliminating the move or copy entirely:

```cpp
PointCloud make_cloud(int n) {
    PointCloud result(n);   // named local variable
    // ... fill it ...
    return result;           // NRVO: compiler constructs result directly where
}                            //       the caller's variable lives — zero copies

PointCloud cloud = make_cloud(1000);   // no move, no copy — one construction total
```

There are two variants:

```cpp
// RVO (unnamed temporary) — guaranteed since C++17, compiler MUST elide
PointCloud make_cloud() {
    return PointCloud(100);   // unnamed — guaranteed zero copies
}

// NRVO (named local) — compiler is allowed but not required to elide
PointCloud make_cloud() {
    PointCloud result(100);   // has a name
    return result;            // usually elided in practice
}
```

NRVO breaks when the compiler can't determine at compile time which object will be returned:

```cpp
PointCloud make_cloud(bool big) {
    PointCloud small_cloud(10);
    PointCloud big_cloud(1000);
    return big ? big_cloud : small_cloud;  // two candidates — can't pre-construct either
}
// Falls back to a move
```

**Practical consequence:** return objects by value freely. You don't need output parameters or heap allocations to avoid copies:

```cpp
// Don't do this "to avoid copies" — NRVO already handles it
void make_cloud(int n, PointCloud* out) { ... }   // unnecessary

// Just do this — idiomatic, clean, equally fast
PointCloud make_cloud(int n) { ... }
```

**Key takeaway:** Mark move constructor and move assignment `noexcept` — `std::vector` and other containers use move only when guaranteed not to throw, so omitting `noexcept` silently forces copying. Return objects by value freely — NRVO eliminates the copy in most cases.

---

## 2.7 RAII (Resource Acquisition Is Initialization)

RAII is the single most important C++ idiom. The rule: **acquire a resource in the constructor, release it in the destructor**. Because destructors run deterministically (scope exit, including exceptions), resource leaks become structurally impossible.

### The Non-RAII Version (Fragile)

```cpp
// BAD: manual resource management — leaks on any early exit.
void process_scan_bad(Lidar& lidar) {
    lidar.activate();    // powers on, starts motor spinning

    auto scan = lidar.get_scan();

    if (scan.empty()) {
        return;          // LEAK: lidar.deactivate() never called, motor keeps spinning
    }

    run_icp(scan);       // if this throws, deactivate() is also skipped

    lidar.deactivate();  // only reached on the happy path
}
```

### The RAII Version (Safe)

```cpp
#include <chrono>
#include <iostream>
#include <string>

// ── Example 1: Scoped sensor activation ─────────────────────────────────────
// Sensor is expensive to run continuously — activate only during a processing
// window, guaranteed to deactivate even if processing throws.

class ScopedSensor {
public:
    explicit ScopedSensor(Lidar& sensor) : sensor_(sensor) {
        sensor_.activate();    // powers on, starts spinning
    }
    ~ScopedSensor() {
        sensor_.deactivate();  // always runs — scope exit, return, or exception
    }
    ScopedSensor(const ScopedSensor&)            = delete;
    ScopedSensor& operator=(const ScopedSensor&) = delete;
private:
    Lidar& sensor_;
};

void process_scan_good(Lidar& lidar) {
    ScopedSensor guard(lidar);   // activate here
    auto scan = lidar.get_scan();
    run_icp(scan);               // if this throws, deactivate still runs
}                                // deactivate here — guaranteed


// ── Example 2: TF2 transform snapshot lock ───────────────────────────────────
// In ROS2, reading multiple transforms without locking the buffer means the
// transform tree can update between lookups — inconsistent state mid-callback.

class ScopedTfSnapshot {
public:
    explicit ScopedTfSnapshot(tf2_ros::Buffer& tf_buffer)
        : tf_buffer_(tf_buffer)
    {
        tf_buffer_.lock();    // freeze the transform tree
    }
    ~ScopedTfSnapshot() {
        tf_buffer_.unlock();  // new transforms can flow in again
    }
    ScopedTfSnapshot(const ScopedTfSnapshot&)            = delete;
    ScopedTfSnapshot& operator=(const ScopedTfSnapshot&) = delete;
private:
    tf2_ros::Buffer& tf_buffer_;
};

void on_lidar_msg(const sensor_msgs::msg::LaserScan& msg) {
    ScopedTfSnapshot snap(tf_buffer_);
    // all lookups here see a consistent, frozen transform tree
    auto tf_map_base  = tf_buffer_.lookupTransform("map",  "base_link", msg.header.stamp);
    auto tf_map_laser = tf_buffer_.lookupTransform("map",  "laser",     msg.header.stamp);
}  // unlock here — even if a lookup throws


// ── Example 3: Scoped diagnostics timer ─────────────────────────────────────
// ROS2 nodes publish processing time via diagnostic_updater.
// RAII ensures the timing result is always published, even on exception.

class ScopedDiagTimer {
public:
    explicit ScopedDiagTimer(const std::string& task,
                             diagnostic_updater::Updater& updater)
        : task_(task)
        , updater_(updater)
        , start_(std::chrono::steady_clock::now())
    {}

    ~ScopedDiagTimer() {
        auto elapsed = std::chrono::steady_clock::now() - start_;
        double ms = std::chrono::duration<double, std::milli>(elapsed).count();
        updater_.broadcast(0, task_ + " took " + std::to_string(ms) + " ms");
    }

private:
    std::string task_;
    diagnostic_updater::Updater& updater_;
    std::chrono::steady_clock::time_point start_;
};

void EkfNode::on_imu_msg(const sensor_msgs::msg::Imu& msg) {
    ScopedDiagTimer timer("imu_callback", updater_);
    ekf_.update(msg);
}  // timing result published here — always


// ── Example 4: Rosbag write guard ───────────────────────────────────────────
// Bag files must be explicitly closed to flush buffers and write the index.
// A crash or early return without close() corrupts the bag.

class ScopedBagWriter {
public:
    explicit ScopedBagWriter(const std::string& path)
        : writer_(std::make_unique<rosbag2_cpp::Writer>())
    {
        writer_->open(path);
    }

    ~ScopedBagWriter() {
        writer_->close();   // flushes buffers and writes index — must not be skipped
    }

    void write(const rclcpp::SerializedMessage& msg,
               const std::string& topic,
               rclcpp::Time stamp) {
        writer_->write(msg, topic, stamp);
    }

    ScopedBagWriter(const ScopedBagWriter&)            = delete;
    ScopedBagWriter& operator=(const ScopedBagWriter&) = delete;

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
};
```

The pattern is identical across all four cases — **acquire in constructor, release in destructor, delete copy**. Only the meaning of acquire/release changes: power on/off, lock/unlock, start/stop timing, open/close bag.

### Why RAII Wrappers Delete Copy

Every example above has these two lines:

```cpp
ScopedSensor(const ScopedSensor&)            = delete;
ScopedSensor& operator=(const ScopedSensor&) = delete;
```

`= delete` tells the compiler: do not generate this function, and make it a **compile error** if anyone tries to use it.

If you allowed copying a `ScopedSensor`, you'd have two objects managing the same sensor:

```cpp
ScopedSensor a(lidar);   // lidar.activate()
ScopedSensor b = a;      // copy — b also holds a reference to lidar

// both destructors run on scope exit:
// b destroyed → lidar.deactivate()
// a destroyed → lidar.deactivate() called AGAIN  ← double-deactivate, bug
```

This is the same class of bug as double-free on a raw pointer. Deleting copy turns it into a compile error instead of a runtime crash:

```cpp
ScopedSensor b = a;   // COMPILE ERROR: use of deleted function
```

**What about move?** Deleting the copy constructor also suppresses the compiler-generated move. For simple scope guards that's fine — you create one, use it, it's gone. If you needed to transfer ownership (e.g. return a guard from a function), you'd explicitly define a move constructor that nulls out the source.

**Key takeaway:** Every resource (sensor power, transform lock, timer, bag file, ROS2 publisher) should be wrapped in a class whose destructor releases it. Delete copy to prevent two wrappers managing the same resource. You get exception safety and deterministic cleanup for free.

---

## 2.8 Smart Pointers

Smart pointers are RAII wrappers around raw pointers. They live in `<memory>`. Prefer them over `new`/`delete` in all new code.

### `unique_ptr` — Sole Ownership

```cpp
#include <memory>
#include <iostream>
#include <string>

class Lidar {
public:
    explicit Lidar(std::string model) : model_{std::move(model)} {
        std::cout << "Lidar " << model_ << " online\n";
    }
    ~Lidar() { std::cout << "Lidar " << model_ << " offline\n"; }
    void spin() { std::cout << model_ << " spinning\n"; }

private:
    std::string model_;
};

void unique_ptr_demo() {
    // make_unique is preferred over new: exception-safe, clear ownership.
    auto lidar = std::make_unique<Lidar>("Velodyne VLP-16");

    lidar->spin();          // arrow operator: same as (*lidar).spin()
    (*lidar).spin();        // dereference also works

    // unique_ptr is NOT copyable.
    // auto lidar2 = lidar;   // compile error

    // unique_ptr IS movable — transfers sole ownership.
    auto lidar2 = std::move(lidar);
    // lidar is now null; lidar2 owns the object.

    if (!lidar) std::cout << "lidar is null after move\n";

    lidar2->spin();
}   // lidar2 destroyed here — Lidar destructor runs automatically
```

### `shared_ptr` — Shared Ownership

```cpp
#include <memory>
#include <iostream>

class Map {
public:
    Map() { std::cout << "Map created\n"; }
    ~Map() { std::cout << "Map destroyed\n"; }
};

void shared_ptr_demo() {
    std::shared_ptr<Map> map1 = std::make_shared<Map>();
    std::cout << "use_count=" << map1.use_count() << "\n";  // 1

    {
        std::shared_ptr<Map> map2 = map1;   // copy: ref count -> 2
        std::cout << "use_count=" << map1.use_count() << "\n";  // 2
        // Both map1 and map2 own the object.
    }   // map2 destroyed: ref count -> 1. Map NOT yet destroyed.

    std::cout << "use_count=" << map1.use_count() << "\n";  // 1
}   // map1 destroyed: ref count -> 0. Map destroyed here.
```

### `weak_ptr` — Non-Owning Observer

`weak_ptr` observes a `shared_ptr` without incrementing the reference count. Use it to break ownership cycles (e.g., parent/child graphs) or to hold a cached reference that might expire.

```cpp
#include <memory>
#include <iostream>

void weak_ptr_demo() {
    std::shared_ptr<int> strong = std::make_shared<int>(42);
    std::weak_ptr<int> weak = strong;   // no ref count increase

    std::cout << "use_count=" << strong.use_count() << "\n";  // 1

    // To use a weak_ptr you must lock() it — produces a temporary shared_ptr.
    if (auto locked = weak.lock()) {
        std::cout << "value=" << *locked << "\n";   // 42
    }

    strong.reset();   // release the shared ownership; object destroyed

    if (weak.expired()) {
        std::cout << "object has been destroyed\n";
    }
}
```

### When to Use Each

| Smart pointer | Use when |
|---|---|
| `unique_ptr` | Default choice. One clear owner. Zero overhead vs raw pointer. |
| `shared_ptr` | Genuinely shared ownership (multiple subsystems need the same object alive). |
| `weak_ptr` | You need to observe a `shared_ptr` without prolonging its life; breaking cycles. |
| Raw pointer `T*` | Non-owning observation of something guaranteed to outlive you (function parameters, iterators). |

```cpp
#include <memory>
#include <vector>
#include <iostream>

struct Plugin {
    virtual ~Plugin() = default;
    virtual void run() = 0;
};

struct EkfPlugin : Plugin {
    void run() override { std::cout << "EKF running\n"; }
};

// Typical robotics pattern: a manager owns plugins via unique_ptr.
class SensorFusion {
public:
    void add_plugin(std::unique_ptr<Plugin> p) {
        plugins_.push_back(std::move(p));
    }
    void run_all() {
        for (auto& p : plugins_) p->run();
    }

private:
    std::vector<std::unique_ptr<Plugin>> plugins_;
};

int main() {
    SensorFusion sf;
    sf.add_plugin(std::make_unique<EkfPlugin>());
    sf.run_all();
}
```

**Key takeaway:** Default to `unique_ptr`. Reach for `shared_ptr` only when you truly cannot determine a single owner. Never use `new`/`delete` in application code.

---

## 2.9 Operator Overloading

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

---

## 2.10 Inheritance & Virtual Functions

Consider a sensor fusion node that reads from wheel odometry, a GPS, and an IMU. Each sensor produces a pose estimate but they all work differently internally. Without inheritance you'd write separate code paths for each:

```cpp
void update(double dt) {
    if (source_type == "wheel")  { update_wheel(dt); }
    else if (source_type == "gnss") { update_gnss(dt); }
    else if (source_type == "imu")  { update_imu(dt); }
    // add a new sensor → edit this function
}
```

Every time you add a sensor you modify existing code. Fragile, and it grows without bound.

Inheritance solves this by letting you define a common interface (`LocalizationSource`) and have each sensor implement it. The fusion loop doesn't need to know which sensor it's talking to:

```cpp
for (auto& source : sources) {
    source->update(dt);   // correct implementation called automatically
}
```

**Virtual functions** are what make this work — they tell the compiler "don't decide which `update()` to call at compile time, decide at runtime based on the actual object type."

---

### Defining the Interface (Abstract Base Class)

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

// Abstract base class — defines what every localization source must be able to do.
// You cannot create a LocalizationSource directly; it only exists to be inherited.
class LocalizationSource {
public:
    // Virtual destructor — explained in detail below.
    virtual ~LocalizationSource() = default;

    // Pure virtual (= 0): no implementation here.
    // Every derived class MUST provide its own version or it won't compile.
    virtual void   update(double dt) = 0;
    virtual std::string name() const = 0;

    // Virtual with a default: derived classes CAN override but don't have to.
    virtual bool is_healthy() const { return true; }

    // Non-virtual: shared behaviour, same for every source, not overridable.
    void log_status() const {
        std::cout << "[" << name() << "] healthy=" << is_healthy() << "\n";
    }
};

// This would be a compile error — LocalizationSource has pure virtual functions.
// LocalizationSource src;
```

---

### Implementing the Interface (Derived Classes)

```cpp
class WheelOdometry : public LocalizationSource {
public:
    explicit WheelOdometry(double wheel_radius)
        : wheel_radius_{wheel_radius} {}

    // 'override' tells the compiler: "I intend to override a virtual function."
    // If you mistype the name or get the signature wrong, the compiler errors —
    // without 'override' it would silently create a brand-new unrelated function.
    void update(double dt) override {
        std::cout << "WheelOdometry: integrating encoders, dt=" << dt << "\n";
    }

    std::string name() const override { return "WheelOdometry"; }

    // is_healthy() not overridden — uses base class default (returns true)

private:
    double wheel_radius_;
};

class GnssLocalization final : public LocalizationSource {
    // 'final' means nothing can inherit from GnssLocalization.
    // Useful when you know the hierarchy ends here — lets the compiler optimise.
public:
    void update(double dt) override {
        std::cout << "GnssLocalization: parsing NMEA fix, dt=" << dt << "\n";
    }

    std::string name() const override { return "GnssLocalization"; }

    // Override is_healthy() — GNSS is only healthy if we have a good fix.
    bool is_healthy() const override { return fix_quality_ > 1; }

private:
    int fix_quality_ = 2;
};
```

---

### Virtual Dispatch at Runtime

This is the payoff. The fusion loop holds a list of base-class pointers and calls `update()` on each — the right implementation runs automatically:

```cpp
void run_fusion_loop() {
    std::vector<std::unique_ptr<LocalizationSource>> sources;
    sources.push_back(std::make_unique<WheelOdometry>(0.1));
    sources.push_back(std::make_unique<GnssLocalization>());

    double dt = 0.05;
    for (auto& src : sources) {
        src->update(dt);     // WheelOdometry::update OR GnssLocalization::update
        src->log_status();   // always LocalizationSource::log_status (non-virtual)
    }
    // Output:
    // WheelOdometry: integrating encoders, dt=0.05
    // [WheelOdometry] healthy=1
    // GnssLocalization: parsing NMEA fix, dt=0.05
    // [GnssLocalization] healthy=1
}
```

Adding a new sensor (`ImuLocalization`) requires zero changes to `run_fusion_loop` — just write the new derived class.

---

### How It Works: the vtable

When a class has at least one virtual function, the compiler adds a hidden pointer (`vptr`) to every object. It points to a table of function pointers specific to that type:

```
WheelOdometry object in memory:
┌──────────────┐
│ vptr ────────┼──→ WheelOdometry vtable
│ wheel_radius_│         [0] → WheelOdometry::update()
└──────────────┘         [1] → WheelOdometry::name()
                         [2] → LocalizationSource::is_healthy()  (not overridden)

GnssLocalization object in memory:
┌──────────────┐
│ vptr ────────┼──→ GnssLocalization vtable
│ fix_quality_ │         [0] → GnssLocalization::update()
└──────────────┘         [1] → GnssLocalization::name()
                         [2] → GnssLocalization::is_healthy()    (overridden)
```

`src->update(dt)` follows the `vptr`, looks up slot `[0]`, calls whatever is there. One extra pointer dereference — negligible cost in practice.

---

### Virtual Destructor

Without a virtual destructor, deleting a derived object through a base pointer only runs the base destructor — the derived destructor is skipped. Resources owned by the derived class leak:

```cpp
// BAD — LocalizationSource has no virtual destructor
LocalizationSource* src = new WheelOdometry(0.1);
delete src;   // only ~LocalizationSource() runs — ~WheelOdometry() is SKIPPED
              // any resources WheelOdometry owns are leaked
              // undefined behaviour
```

With `virtual ~LocalizationSource() = default`, the vtable routes `delete` to the right destructor:

```cpp
// GOOD
LocalizationSource* src = new WheelOdometry(0.1);
delete src;   // virtual dispatch → ~WheelOdometry() runs first, then ~LocalizationSource()
```

**Rule:** if a class has any virtual functions, give it a virtual destructor.

---

### The Slicing Problem

Slicing happens when you copy a derived object into a base object **by value** — the derived-class data and vtable are chopped off:

```cpp
WheelOdometry wo{0.1};

LocalizationSource sliced = wo;   // SLICING: only the LocalizationSource part is copied
                                  // wheel_radius_ is gone, vptr points to base vtable
sliced.update(0.05);              // compile error — update() is pure virtual in base
```

Always use a pointer or reference when working polymorphically:

```cpp
LocalizationSource& ref = wo;    // no slicing — ref looks at the real WheelOdometry
LocalizationSource* ptr = &wo;   // same

ref.update(0.05);   // calls WheelOdometry::update — correct
```

---

### Access Specifiers in Inheritance

```cpp
struct Base {
    int pub  = 1;   // public
protected:
    int prot = 2;   // accessible in derived classes, not from outside
private:
    int priv = 3;   // never accessible in derived classes
};

struct D_pub   : public    Base {};  // pub stays public,    prot stays protected
struct D_prot  : protected Base {};  // pub becomes protected, prot stays protected
struct D_priv  : private   Base {};  // pub becomes private,  prot becomes private
```

In robotics code you almost always use `public` inheritance — it means "is-a". Protected and private inheritance mean "implemented-in-terms-of" and are rarely needed.

---

**Key takeaway:** Define a pure virtual interface in the base class, implement it in derived classes, and always use `override`. Store polymorphic objects as `unique_ptr<Base>` to avoid slicing. Give every polymorphic base class a virtual destructor or derived-class resources will leak silently.

---

## 2.11 static Members & Methods

A `static` member belongs to the **class**, not to any particular instance. All objects of the class share the same static data member.

### Static Data Members

```cpp
#include <iostream>
#include <string>
#include <atomic>

class RosNode {
public:
    explicit RosNode(std::string name) : name_{std::move(name)} {
        ++instance_count_;   // increments the shared counter
        std::cout << "Node " << name_ << " created (total=" << instance_count_ << ")\n";
    }

    ~RosNode() {
        --instance_count_;
        std::cout << "Node " << name_ << " destroyed (total=" << instance_count_ << ")\n";
    }

    // Static member function: no 'this' pointer, can only access static members.
    static int instance_count() { return instance_count_; }

    // Static constant — can be defined inline (integral or constexpr).
    static constexpr int kMaxNodes = 16;

    std::string name() const { return name_; }

private:
    std::string name_;

    // Declaration inside the class.
    // Non-constexpr static data members must be DEFINED outside the class
    // in exactly one .cpp file (see below).
    static int instance_count_;
};

// Definition outside the class (in the .cpp file in a real project).
int RosNode::instance_count_ = 0;

int main() {
    std::cout << "count=" << RosNode::instance_count() << "\n";   // 0

    {
        RosNode n1{"camera_node"};
        RosNode n2{"lidar_node"};
        std::cout << "count=" << RosNode::instance_count() << "\n"; // 2
    }   // both destroyed

    std::cout << "count=" << RosNode::instance_count() << "\n";   // 0
    std::cout << "max="   << RosNode::kMaxNodes << "\n";          // 16
}
```

### Factory Method Pattern

Consider a `Quaternion` class. A quaternion has four components `(w, x, y, z)` but nobody thinks in those terms — you think in axis-angle, Euler angles, or identity. If you expose only a raw four-argument constructor, callers have to know the math just to construct one:

```cpp
// What does this mean? Is it valid? Hard to tell at a glance.
Quaternion q{0.707, 0.0, 0.0, 0.707};
```

A **factory method** is a static function that creates an object with a meaningful name describing *what you're asking for*, not *how it's stored internally*:

```cpp
// Crystal clear — no math knowledge required at the call site.
auto q = Quaternion::from_axis_angle(0.0, 0.0, 1.0, M_PI / 2.0);
```

The static function does the math internally and hands you back a valid object. The constructor can even be made private so callers are forced to go through the factories — guaranteeing the invariant is always established correctly.

```cpp
#include <cmath>
#include <iostream>

class Quaternion {
public:
    double w, x, y, z;

    // Factory: no rotation — the starting point for any rotation sequence.
    static Quaternion identity() {
        return Quaternion{1.0, 0.0, 0.0, 0.0};
    }

    // Factory: rotate by angle_rad around an arbitrary unit axis (ax, ay, az).
    // Caller thinks in geometry — the math stays inside.
    static Quaternion from_axis_angle(double ax, double ay, double az,
                                      double angle_rad)
    {
        double half = angle_rad * 0.5;
        double s    = std::sin(half);
        return Quaternion{std::cos(half), ax*s, ay*s, az*s};
    }

    // Factory: build from a ROS2 geometry_msgs quaternion message.
    // Keeps message-type knowledge isolated in one place.
    static Quaternion from_ros_msg(double mx, double my, double mz, double mw) {
        return Quaternion{mw, mx, my, mz};   // ROS uses (x,y,z,w) order — easy to get wrong
    }

    void print() const {
        std::cout << "Q(w=" << w << ", x=" << x
                  << ", y=" << y << ", z=" << z << ")\n";
    }

private:
    // Private: forces all construction through the named factories above.
    // Nobody can accidentally construct a quaternion with wrong component order.
    Quaternion(double w, double x, double y, double z)
        : w{w}, x{x}, y{y}, z{z} {}
};

int main() {
    auto q1 = Quaternion::identity();
    auto q2 = Quaternion::from_axis_angle(0.0, 0.0, 1.0, M_PI / 2.0);  // 90 deg around Z
    auto q3 = Quaternion::from_ros_msg(0.0, 0.0, 0.707, 0.707);

    q1.print();   // Q(w=1, x=0, y=0, z=0)
    q2.print();   // Q(w=0.707, x=0, y=0, z=0.707)
    q3.print();   // Q(w=0.707, x=0, y=0, z=0.707)

    // Quaternion q{1, 0, 0, 0};  // COMPILE ERROR — constructor is private
}
```

This pattern appears constantly in robotics libraries — `Transform::identity()`, `Pose::from_tf_msg()`, `Covariance::diagonal(...)` — anywhere the internal representation differs from how a human naturally specifies the value.

**Key takeaway:** Use factory methods when the same type can be created from multiple different sources (axis-angle, Euler, ROS message, identity) and you want the call site to read like intent, not math. Making the constructor private enforces that all objects are created through a controlled, named path.

---

## Quick Reference

| Concept | Rule of thumb |
|---|---|
| struct vs class | struct = data, class = invariants |
| explicit | Always on single-arg constructors unless implicit conversion is intended |
| MIL order | Matches declaration order, not MIL order |
| const member fn | Mark anything that doesn't mutate state as const |
| Rule of Three | Destructor + copy ctor + copy assign — define all or none |
| Rule of Five | + move ctor + move assign; mark both noexcept |
| RAII | Resource in constructor, release in destructor |
| Smart pointers | unique_ptr by default; shared_ptr only for genuinely shared ownership |
| Virtual destructor | Required in any base class used polymorphically |
| override | Always use on overriding functions |
| Slicing | Store polymorphic objects through pointers/references, not values |
| static members | Defined in .cpp; static fn has no this |
