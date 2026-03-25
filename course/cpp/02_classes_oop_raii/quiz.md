# Module 2 Quiz — Classes, OOP & RAII

12 questions covering sections 2.1–2.11. Work through each question before expanding the answer.

---

## Conceptual Questions

### Q1 — Rule of Three / Rule of Five

A class `SensorBuffer` owns a raw `float*` array allocated on the heap. You write a custom destructor to `delete[]` it.

(a) Why does the compiler-generated copy constructor cause undefined behaviour for this class?
(b) List the five special member functions that form the Rule of Five.
(c) If you don't need copying at all, what is the cleanest way to express that?

<details>
<summary>Answer</summary>

**(a)** The compiler-generated copy constructor performs a **shallow copy**: it copies the pointer value, not the data it points to. After `SensorBuffer b = a;`, both `a.data_` and `b.data_` point to the same heap allocation. When `b` is destroyed it calls `delete[] b.data_`. When `a` is then destroyed it calls `delete[] a.data_` on the same memory — this is a **double-free**, which is undefined behaviour and typically a crash.

**(b)** The Rule of Five:
1. Destructor
2. Copy constructor
3. Copy assignment operator
4. Move constructor
5. Move assignment operator

**(c)** Delete the copy constructor and copy assignment operator explicitly:

```cpp
SensorBuffer(const SensorBuffer&)            = delete;
SensorBuffer& operator=(const SensorBuffer&) = delete;
```

This makes any accidental copy a **compile error** instead of a runtime crash. You would also typically `= default` or implement the move operations so the object is still transferable.

</details>

---

### Q2 — RAII and Exception Safety

Explain why the following function is not exception-safe, then rewrite it using a RAII wrapper. Your rewrite must guarantee the file is closed even if `process(f)` throws.

```cpp
void read_sensor_log(const char* path) {
    FILE* f = std::fopen(path, "r");
    if (!f) throw std::runtime_error("open failed");

    process(f);    // may throw

    std::fclose(f);
}
```

<details>
<summary>Answer</summary>

**Why it's unsafe:** If `process(f)` throws an exception, the call stack unwinds and `std::fclose(f)` is never reached. The file handle leaks for the lifetime of the process. A `return` statement added before `fclose` for any early-exit path causes the same problem.

**RAII rewrite:**

```cpp
class FileGuard {
public:
    explicit FileGuard(const char* path, const char* mode)
        : file_{std::fopen(path, mode)}
    {
        if (!file_) throw std::runtime_error("open failed");
    }

    ~FileGuard() {
        if (file_) std::fclose(file_);
    }

    FileGuard(const FileGuard&)            = delete;
    FileGuard& operator=(const FileGuard&) = delete;

    FILE* get() { return file_; }

private:
    FILE* file_;
};

void read_sensor_log(const char* path) {
    FileGuard f{path, "r"};   // acquired here
    process(f.get());          // may throw — doesn't matter
}   // FileGuard destructor runs here whether we exit normally or via exception
```

The destructor is guaranteed to run on any scope exit — normal return, exception, or early return. The file cannot leak.

</details>

---

### Q3 — Virtual Destructor

Consider this code:

```cpp
class Sensor {
public:
    ~Sensor() { std::cout << "Sensor dtor\n"; }
    virtual void read() = 0;
};

class Lidar : public Sensor {
public:
    ~Lidar() { std::cout << "Lidar dtor\n"; }
    void read() override { std::cout << "Lidar reading\n"; }
private:
    float* scan_buffer_ = new float[1000];
};

int main() {
    Sensor* s = new Lidar{};
    delete s;
}
```

(a) What output does this program produce, and what resource problem occurs?
(b) What single change fixes it?

<details>
<summary>Answer</summary>

**(a)** Output:

```
Sensor dtor
```

Only `~Sensor()` runs. Because `Sensor::~Sensor` is not `virtual`, the compiler resolves the destructor call statically through the `Sensor*` pointer and calls `~Sensor()` directly — `~Lidar()` is **never called**. The `scan_buffer_` allocated by `Lidar` is never `delete[]`'d, causing a **memory leak**. This is undefined behaviour.

**(b)** Declare the destructor `virtual` in the base class:

```cpp
virtual ~Sensor() { std::cout << "Sensor dtor\n"; }
```

Now `delete s` performs virtual dispatch, calls `~Lidar()` first (which can release `scan_buffer_`), and then calls `~Sensor()`. Output becomes:

```
Lidar dtor
Sensor dtor
```

**Rule:** Any class with at least one virtual function must have a `virtual` destructor.

</details>

---

### Q4 — Operator Overloading: Member vs Free Function

You are implementing a `Pose2D` class with `x`, `y`, and `theta` fields.

(a) Should `operator+=` be a member or a free function? Why?
(b) Should `operator+` (binary addition) be a member or a free function? What advantage does the free form have?
(c) Should `operator<<` (stream output) be a member or a free function? Why can't it be a member of `Pose2D`?

<details>
<summary>Answer</summary>

**(a) `operator+=` — member function.**

The left operand is always a `Pose2D`; no implicit conversion on the left side is needed. Compound assignment operators are conventionally members because they modify `*this` and return a reference to it.

**(b) `operator+` — free function.**

A free function allows implicit conversions on **both** sides of the `+`. If `operator+` were a member, `myPose + something` works (implicit conversion on the right), but `something + myPose` fails if `something` is not a `Pose2D`. As a free function, the compiler can apply conversions to either operand. The canonical pattern is to implement `+` in terms of `+=`:

```cpp
Pose2D operator+(Pose2D lhs, const Pose2D& rhs) {
    lhs += rhs;
    return lhs;
}
```

**(c) `operator<<` — free function.**

The left operand of `<<` is `std::ostream&`, which is not `Pose2D`. A member operator is called as `lhs.operator<<(rhs)`, meaning the left operand's type must be `Pose2D`. Since you cannot modify `std::ostream`, `operator<<` must be a free function (or a friend):

```cpp
std::ostream& operator<<(std::ostream& os, const Pose2D& p) {
    return os << "Pose(" << p.x << ", " << p.y << ", " << p.theta << ")";
}
```

</details>

---

## Predict-the-Output Questions

### Q5 — Copy vs Move and `std::move`

What does this program print? Explain why each line of output appears.

```cpp
#include <iostream>
#include <string>
#include <utility>

class Tag {
public:
    Tag(const char* s) : s_{s} {}

    Tag(const Tag& o)       : s_{o.s_}            { std::cout << "copy " << s_ << "\n"; }
    Tag(Tag&& o) noexcept   : s_{std::move(o.s_)} { std::cout << "move " << s_ << "\n"; }

    ~Tag() { std::cout << "dtor " << s_ << "\n"; }

    std::string s_;
};

Tag make_tag() {
    Tag t{"sensor"};
    return t;
}

int main() {
    Tag a{"alpha"};
    Tag b = a;
    Tag c = std::move(a);
    Tag d = make_tag();
}
```

<details>
<summary>Answer</summary>

```
copy alpha
move alpha
move sensor       <- may be elided; see note
dtor sensor
dtor alpha
dtor alpha
dtor alpha
```

**Line by line:**

1. `Tag a{"alpha"}` — calls the `const char*` converting constructor. No output (no copy/move).
2. `Tag b = a` — `a` is an lvalue, so the **copy constructor** is called: prints `copy alpha`.
3. `Tag c = std::move(a)` — `std::move(a)` casts `a` to an rvalue reference. The **move constructor** is selected: prints `move alpha`. After this, `a.s_` is in a valid but unspecified state (typically empty string).
4. `Tag d = make_tag()` — the compiler is likely to apply **NRVO** (Named Return Value Optimisation), constructing `t` directly into `d`'s storage. If NRVO fires, there is **no copy or move** printed. If NRVO is suppressed (e.g., `-fno-elide-constructors`), the move constructor fires: `move sensor`.

**Destructor order** (LIFO on scope exit): `d`, `c`, `b`, `a` — four `dtor` lines. `c.s_` is `"alpha"` (moved from), `a.s_` is likely `""` after the move, but the destructor still runs on `a`.

**Note:** Exact output for `d` depends on whether NRVO is applied. In practice with optimisations, no copy/move line appears for `make_tag()`.

</details>

---

### Q6 — Member Initializer List Order

What does this program print? Explain the surprising result.

```cpp
#include <iostream>

class Range {
public:
    Range(int low, int high)
        : size_{high - low}   // MIL entry 1
        , low_{low}           // MIL entry 2
        , high_{high}         // MIL entry 3
    {}

    void print() const {
        std::cout << "low="  << low_
                  << " high=" << high_
                  << " size=" << size_ << "\n";
    }

private:
    int low_;     // declared first
    int high_;    // declared second
    int size_;    // declared third
};

int main() {
    Range r{2, 10};
    r.print();
}
```

<details>
<summary>Answer</summary>

```
low=2 high=10 size=8
```

Wait — is this actually surprising? Let's look more carefully.

Members are initialized in **declaration order**: `low_`, then `high_`, then `size_`. The MIL lists them in a different order (`size_` first, then `low_`, then `high_`), but the initialization order follows the **declaration order** regardless.

So the actual initialization sequence is:
1. `low_`  = `low` (= 2) ✓
2. `high_` = `high` (= 10) ✓
3. `size_` = `high - low` (= 8) ✓

This works correctly here because `size_` is initialized last (as declared) using the constructor parameters `high` and `low` — not using `high_` or `low_`. If `size_` were initialized using `high_ - low_` (the member variables rather than the parameters), and if `size_` were **declared before** `low_` and `high_`, then `size_` would use the uninitialized members and produce undefined behaviour.

**The lesson:** The MIL order that matters is **declaration order**. Write the MIL in declaration order and use compiler flag `-Wreorder` to catch mismatches. Never initialize a member using another member that is declared after it.

</details>

---

### Q7 — Abstract Class and Slicing

What does this program print, or does it fail to compile? Explain.

```cpp
#include <iostream>

class Shape {
public:
    virtual double area() const = 0;
    virtual ~Shape() = default;
    void describe() const {
        std::cout << "area=" << area() << "\n";
    }
};

class Square : public Shape {
public:
    explicit Square(double s) : side_{s} {}
    double area() const override { return side_ * side_; }
private:
    double side_;
};

void print_area(Shape s) {   // takes Shape by VALUE
    s.describe();
}

int main() {
    Square sq{4.0};
    print_area(sq);
}
```

<details>
<summary>Answer</summary>

**This does not compile.**

`print_area` takes a `Shape` by value. To construct the parameter, the compiler must copy a `Square` into a `Shape` object. But `Shape` has a pure virtual function (`area() = 0`), making it an **abstract class**. Abstract classes cannot be instantiated — you cannot create an object of type `Shape`. The compiler rejects the copy.

Additionally, even if `Shape` were concrete, this would be the **slicing problem**: copying a `Square` into a `Shape` would discard `side_` and the vtable pointer for `Square`, leaving a useless truncated object.

**Fix:** Pass by const reference or pointer to preserve polymorphism and avoid slicing:

```cpp
void print_area(const Shape& s) {
    s.describe();   // virtual dispatch works correctly through reference
}
```

</details>

---

## Find-the-Bug Questions

### Q8 — Deep Copy Bug

The following class is supposed to manage a named array of doubles. Find all the bugs and explain how to fix each one.

```cpp
class NamedArray {
public:
    NamedArray(const char* name, std::size_t size)
        : name_{name}, size_{size}, data_{new double[size]}
    {}

    ~NamedArray() { delete[] data_; }

    double& operator[](std::size_t i) { return data_[i]; }

    void print() const {
        std::cout << name_ << ": ";
        for (std::size_t i = 0; i < size_; ++i)
            std::cout << data_[i] << " ";
        std::cout << "\n";
    }

private:
    const char* name_;
    std::size_t size_;
    double*     data_;
};

int main() {
    NamedArray a{"accel", 3};
    a[0] = 1.0; a[1] = 2.0; a[2] = 3.0;

    NamedArray b = a;   // copy
    b[0] = 99.0;

    a.print();   // should print: accel: 1 2 3
    b.print();   // should print: accel: 99 2 3
}
```

<details>
<summary>Answer</summary>

**Bug 1 — Shallow copy: missing copy constructor.**

The compiler-generated copy constructor copies `data_` by pointer value. After `NamedArray b = a`, both `a.data_` and `b.data_` point to the same heap array. `b[0] = 99.0` modifies both arrays. When either object is destroyed, `delete[] data_` runs. When the second is destroyed, it runs again on already-freed memory — **double-free, undefined behaviour**.

**Fix:** Write a copy constructor that allocates a new array and copies the data:

```cpp
NamedArray(const NamedArray& o)
    : name_{o.name_}, size_{o.size_}, data_{new double[o.size_]}
{
    std::copy(o.data_, o.data_ + o.size_, data_);
}
```

**Bug 2 — Missing copy assignment operator.**

Same problem applies to `b = a` (assignment after construction). Must implement copy assignment with a self-assignment guard.

```cpp
NamedArray& operator=(const NamedArray& o) {
    if (this == &o) return *this;
    double* tmp = new double[o.size_];
    std::copy(o.data_, o.data_ + o.size_, tmp);
    delete[] data_;
    data_ = tmp;
    size_ = o.size_;
    name_ = o.name_;
    return *this;
}
```

**Bug 3 — `const char*` name ownership (minor).**

`name_` stores a raw `const char*` pointing into a string literal or caller-owned buffer. If the caller's buffer goes out of scope, `name_` is dangling. Use `std::string` instead:

```cpp
std::string name_;
```

**Summary:** This class violates the Rule of Three. Defining a destructor without also defining copy constructor and copy assignment operator is the bug. As a better long-term fix, replace the raw pointer with `std::vector<double>` and let it handle all memory management automatically.

</details>

---

### Q9 — `explicit` and Implicit Conversion Bug

The following code compiles and runs, but produces a silent logic bug. Identify it and fix it.

```cpp
#include <iostream>

class Radians {
public:
    Radians(double r) : r_{r} {}
    double value() const { return r_; }
private:
    double r_;
};

class Degrees {
public:
    Degrees(double d) : d_{d} {}
    double value() const { return d_; }
private:
    double d_;
};

void set_heading(Radians heading) {
    std::cout << "heading set to " << heading.value() << " rad\n";
}

int main() {
    Degrees user_input{90.0};
    set_heading(user_input.value());   // pass the raw double
}
```

<details>
<summary>Answer</summary>

**The bug:** `user_input.value()` returns `90.0` (degrees). `set_heading` expects a `Radians`. The compiler silently converts `90.0` (a `double`) to `Radians{90.0}` using `Radians`'s non-`explicit` constructor. The function receives 90 radians — roughly 5160 degrees — instead of the intended π/2 radians. There is no warning.

**Fix:** Mark `Radians(double)` as `explicit`. This makes the implicit conversion a compile error, forcing the caller to be deliberate:

```cpp
class Radians {
public:
    explicit Radians(double r) : r_{r} {}
    ...
};
```

Now `set_heading(user_input.value())` is a **compile error**. The programmer must write:

```cpp
// Option A: explicit construction
set_heading(Radians{user_input.value() * M_PI / 180.0});

// Option B: add a proper conversion function
set_heading(Radians{degrees_to_radians(user_input.value())});
```

**Rule:** Whenever a single-argument constructor could silently accept a value from a semantically different type (degrees vs radians, metres vs feet, Hz vs rad/s), mark it `explicit`.

</details>

---

### Q10 — Static Member Definition Bug

This code compiles but fails to link. What is wrong and how do you fix it?

```cpp
// robot.hpp
#pragma once
#include <string>
#include <iostream>

class Robot {
public:
    explicit Robot(std::string name) : name_{std::move(name)} {
        ++count_;
    }
    ~Robot() { --count_; }

    static int count() { return count_; }

private:
    std::string name_;
    static int count_;   // declaration only
};

// main.cpp
#include "robot.hpp"

int main() {
    Robot r1{"r2d2"};
    Robot r2{"c3po"};
    std::cout << Robot::count() << "\n";
}
```

<details>
<summary>Answer</summary>

**The bug:** `static int count_` inside the class body is a **declaration**, not a definition. The linker cannot find the storage for `count_` because it has never been defined. The error is typically:

```
undefined reference to `Robot::count_'
```

**Fix:** Add the definition in exactly one `.cpp` file (outside the class):

```cpp
// robot.cpp
#include "robot.hpp"

int Robot::count_ = 0;   // definition — allocates storage and sets initial value
```

**Alternative (C++17 and later):** Declare it `inline static` inside the class, which serves as both declaration and definition:

```cpp
inline static int count_ = 0;   // C++17 — no separate .cpp definition needed
```

Or use `static constexpr` for compile-time constants, which has also been inline-defined since C++17:

```cpp
static constexpr int kMaxRobots = 10;   // inline definition, no .cpp needed
```

</details>

---

## Code-Completion Questions

### Q11 — Move Constructor and Move Assignment

Complete the two missing special member functions for this class. The class owns a heap-allocated array. Your implementations must:
- Transfer ownership without allocating new memory
- Leave the source object in a valid, destructible state
- Be marked `noexcept`

```cpp
class FloatBuffer {
public:
    explicit FloatBuffer(std::size_t n)
        : size_{n}, data_{new float[n]}
    {}

    ~FloatBuffer() { delete[] data_; }

    FloatBuffer(const FloatBuffer& o)
        : size_{o.size_}, data_{new float[o.size_]}
    {
        std::copy(o.data_, o.data_ + o.size_, data_);
    }

    FloatBuffer& operator=(const FloatBuffer& o) {
        if (this == &o) return *this;
        float* tmp = new float[o.size_];
        std::copy(o.data_, o.data_ + o.size_, tmp);
        delete[] data_;
        data_ = tmp;
        size_ = o.size_;
        return *this;
    }

    // TODO: move constructor
    FloatBuffer(FloatBuffer&& o) noexcept
        /* your implementation here */

    // TODO: move assignment operator
    FloatBuffer& operator=(FloatBuffer&& o) noexcept
        /* your implementation here */

    std::size_t size() const { return size_; }
    float* data()            { return data_; }

private:
    std::size_t size_;
    float*      data_;
};
```

<details>
<summary>Answer</summary>

```cpp
// Move constructor — steal the source's resources, null it out.
FloatBuffer(FloatBuffer&& o) noexcept
    : size_{o.size_}, data_{o.data_}
{
    o.data_ = nullptr;   // source no longer owns the memory
    o.size_ = 0;         // leave source in a valid, empty state
}

// Move assignment operator.
FloatBuffer& operator=(FloatBuffer&& o) noexcept {
    if (this == &o) return *this;   // self-move guard (rare but valid)

    delete[] data_;       // release our current resource

    data_   = o.data_;    // steal source's pointer
    size_   = o.size_;

    o.data_ = nullptr;    // null out source
    o.size_ = 0;

    return *this;
}
```

**Key points:**
- Setting `o.data_ = nullptr` is essential. When `o` is destroyed, its destructor calls `delete[] nullptr`, which is a no-op — safe.
- `noexcept` is critical: `std::vector` uses move only when it is `noexcept`. Without it, reallocations fall back to copying.
- The self-move guard (`this == &o`) is technically required by the standard (moved-from state must be valid), though self-move is uncommon in practice.

</details>

---

### Q12 — Abstract Class Interface

Complete the `LocalizationFilter` abstract base class and one concrete derived class. Fill in the three marked sections.

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

// Abstract base: defines the interface for all state estimators.
class LocalizationFilter {
public:
    // TODO 1: Declare a virtual destructor so derived classes
    //         are destroyed correctly through a base pointer.


    // TODO 2: Declare two pure virtual functions:
    //   - predict(double dt)  — propagates state forward by dt seconds
    //   - update(double meas) — incorporates a measurement
    //   Both take a double and return void.


    // Non-virtual utility: calls predict then update.
    void step(double dt, double meas) {
        predict(dt);
        update(meas);
    }

    virtual std::string name() const = 0;
};

// TODO 3: Implement a concrete class KalmanFilter that:
//   - inherits publicly from LocalizationFilter
//   - has a constructor taking (double process_noise, double meas_noise)
//   - overrides predict, update, and name
//   - stores a double state_ (initialised to 0.0) and prints what it's doing


// --- Driver (do not modify) ---
int main() {
    std::vector<std::unique_ptr<LocalizationFilter>> filters;
    filters.push_back(std::make_unique<KalmanFilter>(0.1, 0.5));

    for (auto& f : filters) {
        f->step(0.05, 3.14);
        std::cout << f->name() << " state processed\n";
    }
}
```

<details>
<summary>Answer</summary>

```cpp
#include <iostream>
#include <string>
#include <memory>
#include <vector>

class LocalizationFilter {
public:
    // TODO 1: Virtual destructor.
    virtual ~LocalizationFilter() = default;

    // TODO 2: Pure virtual functions.
    virtual void predict(double dt)    = 0;
    virtual void update(double meas)   = 0;

    void step(double dt, double meas) {
        predict(dt);
        update(meas);
    }

    virtual std::string name() const = 0;
};

// TODO 3: Concrete derived class.
class KalmanFilter : public LocalizationFilter {
public:
    KalmanFilter(double process_noise, double meas_noise)
        : process_noise_{process_noise}
        , meas_noise_{meas_noise}
        , state_{0.0}
    {}

    void predict(double dt) override {
        // Simplified: just drift state by a tiny amount.
        state_ += process_noise_ * dt;
        std::cout << "KalmanFilter predict dt=" << dt
                  << " state=" << state_ << "\n";
    }

    void update(double meas) override {
        // Simplified Kalman update: blend state toward measurement.
        double gain = process_noise_ / (process_noise_ + meas_noise_);
        state_ += gain * (meas - state_);
        std::cout << "KalmanFilter update meas=" << meas
                  << " state=" << state_ << "\n";
    }

    std::string name() const override { return "KalmanFilter"; }

private:
    double process_noise_;
    double meas_noise_;
    double state_;
};

int main() {
    std::vector<std::unique_ptr<LocalizationFilter>> filters;
    filters.push_back(std::make_unique<KalmanFilter>(0.1, 0.5));

    for (auto& f : filters) {
        f->step(0.05, 3.14);
        std::cout << f->name() << " state processed\n";
    }
}
```

**What to notice:**
- `virtual ~LocalizationFilter() = default` ensures that `delete` through a `LocalizationFilter*` calls `~KalmanFilter()` correctly.
- `= 0` on `predict` and `update` makes `LocalizationFilter` abstract — you cannot instantiate it directly.
- `override` on each derived method causes a compile error if the signature doesn't match a base virtual function.
- The vector stores `unique_ptr<LocalizationFilter>` — polymorphism via pointer, no slicing.

</details>
