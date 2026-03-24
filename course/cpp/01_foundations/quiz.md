# Module 1 Quiz — Foundations & Toolchain

Attempt every question before expanding the answers.

---

## Part A — Written Questions

**Q1.** What is the difference between a compiler error and a linker error? Give one example of each.

<details>
<summary>Answer</summary>

**Compiler error** — caught during compilation of a single `.cpp` file. The compiler checks syntax and types but knows nothing about other translation units.
Example: calling a function with wrong argument types, using an undeclared variable.

**Linker error** — caught when combining `.o` files. The linker cannot find a definition that was declared but never implemented.
Example: `undefined reference to 'compute_hypotenuse(double, double)'` — the declaration was in a header but no `.o` file contained the definition.

</details>

---

**Q2.** You put the definition of a free function in a header and include it in three `.cpp` files. What happens at link time and why?

<details>
<summary>Answer</summary>

The linker sees three copies of the same symbol (one from each `.o` file) and raises a **multiple definition** error. Each `.cpp` that includes the header gets its own copy after preprocessing. The linker requires exactly one definition per symbol.

Fix: move the definition to a `.cpp` file, or mark the function `inline`.

</details>

---

**Q3.** What does `#pragma once` do? Why must it appear at the top of a header?

<details>
<summary>Answer</summary>

It tells the compiler to include this file at most once per translation unit, preventing redefinition errors from repeated inclusion (direct or transitive). It must appear before any code so it takes effect before any declarations are processed.

</details>

---

**Q4.** Explain the difference between `const int* p` and `int* const p`.

<details>
<summary>Answer</summary>

`const int* p` — pointer to const int. Cannot modify `*p`. Can reassign `p` to point elsewhere.

`int* const p` — const pointer to int. Can modify `*p`. Cannot reassign `p`.

Read right-to-left: `const int* p` → "p is a pointer to const int".

</details>

---

**Q5.** Why should you never write `using namespace std;` in a header file?

<details>
<summary>Answer</summary>

`#include` pastes the header into every file that includes it. The `using namespace` then silently applies to all those translation units, potentially pulling in names that clash with names in those files or other headers. This can cause ambiguous overloads or silent changes in which function gets called.

</details>

---

## Part B — Predict the Output

**Q6.** What does this print?

```cpp
#include <iostream>
#include <cstdint>
int main() {
    uint8_t x = 200;
    uint8_t y = 100;
    uint8_t z = x + y;
    std::cout << static_cast<int>(z) << "\n";
}
```

<details>
<summary>Answer</summary>

**44** — `200 + 100 = 300`. `uint8_t` holds 0–255. `300 % 256 = 44`. Silent wraparound.

</details>

---

**Q7.** What does this print?

```cpp
#include <iostream>
void increment(int x) { x += 1; }
int main() {
    int a = 5;
    increment(a);
    std::cout << a << "\n";
}
```

<details>
<summary>Answer</summary>

**5** — `increment` receives a copy of `a`. The original is not modified. To modify the caller's variable, take `int&` or `int*`.

</details>

---

**Q8.** What does this print?

```cpp
#include <iostream>
namespace A { int x = 1; }
namespace B { int x = 2; }
int main() {
    using namespace A;
    std::cout << x << "\n";
    {
        using namespace B;
        std::cout << x << "\n";
    }
    std::cout << x << "\n";
}
```

<details>
<summary>Answer</summary>

Prints `1`, then a **compile error** on the second `cout`. Inside the inner block both `A::x` and `B::x` are in scope — the compiler cannot choose between them (ambiguous). The third line would print `1` if the program compiled.

</details>

---

## Part C — Find the Bug

**Q9.** Find the bug and explain what happens at runtime.

```cpp
void print_first(const int* arr, size_t size) {
    std::cout << arr[0] << "\n";
}
int main() {
    int* p = nullptr;
    print_first(p, 0);
}
```

<details>
<summary>Answer</summary>

Null pointer dereference. `p` is `nullptr`. `arr[0]` dereferences it unconditionally — undefined behaviour, typically a segfault. Fix: guard with `if (!arr || size == 0) return;`.

</details>

---

**Q10.** This CMakeLists.txt fails to build. Find and fix the mistake.

```cmake
add_library(sensor src/sensor.cpp)
add_executable(demo src/main.cpp)
target_include_directories(sensor PRIVATE include)
target_link_libraries(demo PRIVATE sensor)
```

`main.cpp` includes `"sensor.hpp"` from `include/` but the build fails with "file not found".

<details>
<summary>Answer</summary>

`PRIVATE` means the include path is only available when compiling `sensor.cpp` — it is not propagated to `demo`. Fix: change to `PUBLIC`.

```cmake
target_include_directories(sensor PUBLIC include)
```

Rule: if your library's headers are part of its public API (included by consumers), use `PUBLIC`.

</details>

---

## Part D — Code Completion

**Q11.** Complete this function without using any library function other than `std::acos` and `std::sqrt`.

```cpp
double angle_between(double ax, double ay, double bx, double by) {
    // TODO
}
```

<details>
<summary>Answer</summary>

```cpp
double angle_between(double ax, double ay, double bx, double by) {
    double dot   = ax*bx + ay*by;
    double mag_a = std::sqrt(ax*ax + ay*ay);
    double mag_b = std::sqrt(bx*bx + by*by);
    double cos_theta = dot / (mag_a * mag_b);
    // clamp to [-1,1] — floating-point can exceed bounds and cause NaN
    cos_theta = cos_theta < -1.0 ? -1.0 : (cos_theta > 1.0 ? 1.0 : cos_theta);
    return std::acos(cos_theta);
}
```

</details>

---

**Q12.** Write the CMakeLists.txt for this layout. C++17, library named `filters_lib`, executable named `robot_filter`. Both library sources and `main.cpp` need `include/`.

```
project/
├── CMakeLists.txt
├── include/filters.hpp
└── src/
    ├── filters.cpp
    ├── ekf.cpp
    └── main.cpp
```

<details>
<summary>Answer</summary>

```cmake
cmake_minimum_required(VERSION 3.15)
project(robot_filter)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(filters_lib src/filters.cpp src/ekf.cpp)
target_include_directories(filters_lib PUBLIC include)

add_executable(robot_filter src/main.cpp)
target_link_libraries(robot_filter PRIVATE filters_lib)
```

</details>

---

## Score

| Score | Next step |
|-------|-----------|
| 12/12 | Move to Module 2 |
| 9–11  | Review missed concepts, then proceed |
| < 9   | Re-read concepts.md sections for missed questions |

---

## Module 1 Project — 2D Vector Library

Complete the `Vector2d` library from exercise 05, then extend it:

1. All six original operations passing expected output
2. `Vector2d::rotated(double angle_rad) const` — returns vector rotated counterclockwise
3. `double angle_to(const Vector2d& other) const` — angle between vectors
4. `operator<<` so `std::cout << v` prints `(x, y)`
