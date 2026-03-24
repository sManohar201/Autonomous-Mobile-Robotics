# Module 1 — Foundations & Toolchain

> **Before starting:** these concepts are the ground floor. Everything in every robotics codebase — from `robot_localization` to `slam_toolbox` — sits on top of these.

---

## 1.1 The Compilation Pipeline

When you run `g++ main.cpp -o program`, four things happen in sequence.

### Step 1 — Preprocessor

The preprocessor runs before compilation. It handles all lines starting with `#`. It does text substitution — it knows nothing about C++ syntax.

```cpp
#include <iostream>      // copy-paste the entire iostream header here
#define PI 3.14159       // replace every occurrence of PI with 3.14159

int main() {
    double area = PI * 5 * 5;   // becomes: double area = 3.14159 * 5 * 5;
}
```

See the preprocessed output:
```bash
g++ -E main.cpp -o main.i
```

### Step 2 — Compiler

Translates preprocessed C++ into object code (`.o` files). Each `.cpp` compiles independently. The compiler checks syntax and types but does **not** resolve references between files.

```bash
g++ -c main.cpp -o main.o    # compile only, do not link
```

If `main.cpp` calls `void foo()` defined in `utils.cpp`, the compiler accepts it — it leaves a placeholder for the linker.

### Step 3 — Linker

Combines `.o` files and libraries into an executable. Resolves all placeholders.

```bash
g++ main.o utils.o -o program
```

**"undefined reference" errors come from here.** The compiler compiled fine. The linker cannot find the definition.

### Step 4 — Executable

The OS loads the executable, maps it into memory, and jumps to `main()`.

### Summary

```
main.cpp  →  [Preprocessor]  →  main.i  →  [Compiler]  →  main.o  ─┐
utils.cpp →  [Preprocessor]  →  utils.i →  [Compiler]  →  utils.o ─┤→ [Linker] → program
```

---

## 1.2 Headers vs Source Files

| File | Contains | Why |
|------|----------|-----|
| `.hpp` header | **Declarations** | Tells the compiler what exists |
| `.cpp` source | **Definitions** | The actual code, compiled once |

A **declaration** says "this exists, here is its type":
```cpp
double dot_product(double ax, double ay, double bx, double by);
```

A **definition** provides the body:
```cpp
double dot_product(double ax, double ay, double bx, double by) {
    return ax * bx + ay * by;
}
```

If you put a definition in a header and include it from two `.cpp` files, both `.o` files contain a copy — the linker sees two definitions of the same symbol and errors: `multiple definition of 'dot_product'`.

### What belongs in headers

```cpp
// OK in headers
class Sensor { ... };                              // class definition
double dot(double a, double b);                    // function declaration
inline double sq(double x) { return x*x; }        // inline function
template<typename T> T clamp(T v, T lo, T hi);    // template (must be in header)
constexpr double PI = 3.14159265358979;            // constexpr constant
```

```cpp
// NOT in headers (causes multiple definition)
double dot(double a, double b) { return a * b; }  // non-inline definition
int counter = 0;                                   // non-const variable
```

---

## 1.3 Include Guards and `#pragma once`

Without a guard, including `b.hpp` through two paths causes its contents to appear twice — redefinition error.

### Include guards (portable)
```cpp
#ifndef MYPROJECT_B_HPP
#define MYPROJECT_B_HPP
// ... contents ...
#endif
```

### `#pragma once` (simpler, universally supported)
```cpp
#pragma once
// ... contents ...
```

Use `#pragma once` for new code. All major robotics codebases use it.

---

## 1.4 Basic Types

| Type | Size | Use |
|------|------|-----|
| `bool` | 1 byte | true / false |
| `int` | 4 bytes | general integer |
| `double` | 8 bytes | double-precision float — use for physics |
| `float` | 4 bytes | single-precision — use for sensor data |
| `size_t` | 8 bytes | sizes and indices |

### Fixed-width types (`<cstdint>`)
```cpp
#include <cstdint>
uint8_t   byte_val = 255;     // 0 to 255
int16_t   short_val = -1000;
uint32_t  count = 0;
int64_t   timestamp_ns = 0;   // nanoseconds since epoch
```

### Implicit conversion and narrowing
```cpp
double d = 3.99;
int    i = d;           // truncates to 3 — silent data loss
uint8_t b = 300;        // wraps to 44 — silent data loss

// Use explicit casts when you intend conversion:
int i2 = static_cast<int>(d);
```

---

## 1.5 `const`

### `const` variables
```cpp
const double GRAVITY = 9.81;
GRAVITY = 10.0;   // compile error
```

### `const` with pointers — two distinct meanings
```cpp
int x = 5, y = 10;

const int* p1 = &x;   // pointer to const int
*p1 = 20;             // error: cannot modify value
p1  = &y;             // OK: can point elsewhere

int* const p2 = &x;   // const pointer to int
*p2 = 20;             // OK: can modify value
p2  = &y;             // error: cannot repoint
```

Read right-to-left: `const int* p` = "p is a pointer to const int".

### `const` references for function parameters
```cpp
void process(const std::vector<double>& data) { ... }  // no copy
```

Rule: pass cheap types (int, double) by value. Pass everything else by `const&`.

### `const` member functions
```cpp
class Sensor {
    double value_;
public:
    double read() const { return value_; }   // does not modify object
    void   set(double v) { value_ = v; }
};
```

Mark every read-only member function `const`. The compiler enforces it.

---

## 1.6 References vs Pointers

```cpp
int x = 42;
int* ptr = &x;    // pointer: stores the address
int& ref = x;     // reference: another name for x

*ptr = 10;        // modify through pointer
ref  = 10;        // modify through reference — no * needed
```

| Property | Reference | Pointer |
|----------|-----------|---------|
| Can be null | No | Yes (`nullptr`) |
| Can be reseated | No | Yes |
| Must be initialised | Yes | No |
| Arithmetic | No | Yes |

**Use a reference when:** the thing is guaranteed to exist (function parameters, range-for).
**Use a pointer when:** null is a valid state, or you need to swap what you're pointing to.

Never dereference a pointer without checking it:
```cpp
void process(const Scan* scan) {
    if (!scan) return;
    // safe to use scan->...
}
```

---

## 1.7 Function Overloading

Multiple functions with the same name, different parameter types. The compiler picks at compile time — no runtime cost.

```cpp
double magnitude(double x, double y)           { return std::sqrt(x*x + y*y); }
double magnitude(double x, double y, double z) { return std::sqrt(x*x + y*y + z*z); }

magnitude(3.0, 4.0);          // calls 2D version
magnitude(1.0, 2.0, 2.0);     // calls 3D version
```

Compiler prefers exact match over implicit conversion. If two conversions are equally good: ambiguous → compile error.

---

## 1.8 Namespaces

```cpp
namespace math {
    double dot(double ax, double ay, double bx, double by);
}

// Usage options:
double d = math::dot(1.0, 0.0, 0.0, 1.0);   // fully qualified — always safe

using math::dot;      // imports one name — OK inside a function
using namespace math; // imports everything — NEVER in a header
```

**Never `using namespace X;` in a header.** It pollutes every file that includes it.

### Anonymous namespaces — file-local linkage
```cpp
namespace {
    double clamp01(double x) { ... }  // invisible outside this .cpp
}
```

---

## 1.9 CMake Basics

```cmake
cmake_minimum_required(VERSION 3.15)
project(robot_math)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(mathutils src/mathutils.cpp)
target_include_directories(mathutils PUBLIC include)

add_executable(main_program src/main.cpp)
target_link_libraries(main_program PRIVATE mathutils)
```

Build:
```bash
cmake -B build
cmake --build build
./build/main_program
```

### `PUBLIC` vs `PRIVATE` vs `INTERFACE`

| Keyword | Meaning |
|---------|---------|
| `PRIVATE` | Only this target |
| `PUBLIC` | This target AND anything linking it |
| `INTERFACE` | Only things linking this target (not the target itself) |

If your library's headers are included by consumers, use `PUBLIC` on `target_include_directories`.

### Build types
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug      # -g, no optimisation
cmake -B build -DCMAKE_BUILD_TYPE=Release    # -O3, no debug symbols
cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo  # -O2 + -g
```

---

## Key Takeaways

1. **Pipeline**: preprocessor → compiler → linker. "Undefined reference" = linker error.
2. **Headers = declarations, source = definitions.** Definitions in headers cause multiple-definition errors.
3. **`#pragma once`** in every header.
4. **Use `double`** for physics. Use fixed-width types (`int64_t`, `uint8_t`) when size matters.
5. **`const` is correctness**, not style. Mark parameters, member functions, and variables `const` wherever possible.
6. **`const&` for function parameters** — no copy.
7. **References can't be null.** Use pointers when null is valid.
8. **Never `using namespace X;` in a header.**
9. **CMake `PUBLIC`/`PRIVATE`** — get this right or consumers won't find your headers.

---

→ [Exercises](exercises/) → [Quiz](quiz.md)
