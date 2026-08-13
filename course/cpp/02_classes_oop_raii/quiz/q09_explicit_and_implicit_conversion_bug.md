# Q9 — `explicit` and Implicit Conversion Bug

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
