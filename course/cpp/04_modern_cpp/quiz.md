# Module 4 Quiz - Modern C++

12 questions covering sections 4.1-4.15. Work through each question before expanding the answer.

---

## Conceptual Questions

### Q1 - `auto` and References

Given:

```cpp
const std::string name = "imu";
auto a = name;
auto& b = name;
const auto& c = name;
```

What are the types of `a`, `b`, and `c`?

<details>
<summary>Answer</summary>

`a` is `std::string`. Plain `auto` drops top-level `const` and copies the value.

`b` is `const std::string&`. Because `name` is const, a non-const reference cannot bind to it; `auto&` deduces the referenced type as const.

`c` is `const std::string&`.

</details>

---

### Q2 - Structured Binding Copies

Why can this loop accidentally copy more than intended?

```cpp
for (auto [id, scan] : scan_map) {
    process(scan);
}
```

How should it be written if `scan` is large and read-only?

<details>
<summary>Answer</summary>

`auto [id, scan]` copies each key/value pair member into local variables. If `scan` owns a large vector, every iteration performs an expensive copy.

Use a const reference binding:

```cpp
for (const auto& [id, scan] : scan_map) {
    process(scan);
}
```

</details>

---

### Q3 - Lambda Captures

Why is `[&]` risky in a callback that may run later?

<details>
<summary>Answer</summary>

`[&]` captures all used local variables by reference. If the callback is stored and invoked after the surrounding function returns, those references dangle. This is common in asynchronous robotics code: timers, subscriptions, futures, and thread pools.

Prefer explicit captures and make lifetime obvious. Capture values when possible; capture `this` only when object lifetime is guaranteed.

</details>

---

### Q4 - `std::function`

What does `std::function<void(double)>` store, and what is the tradeoff?

<details>
<summary>Answer</summary>

It stores any callable that can be invoked with one `double` argument and returns `void`: a function pointer, lambda, functor, or bound member function.

The tradeoff is type-erased dispatch and possible allocation. It is useful at callback boundaries, but templates or concrete callable types are better for hot inner loops.

</details>

---

### Q5 - `std::optional`

When is `std::optional<T>` a better return type than returning `T` with a sentinel value?

<details>
<summary>Answer</summary>

Use `optional` when absence is expected and recoverable. It makes "no value" explicit in the type system.

Sentinel values are fragile because they can overlap with valid values or be forgotten by callers.

</details>

---

### Q6 - `std::variant`

When is `std::variant` a better fit than a virtual base class?

<details>
<summary>Answer</summary>

Use `variant` when the set of possible alternatives is fixed and known at compile time. It keeps storage value-based and makes handling cases explicit through `std::visit`.

Use a virtual base class when alternatives need to be extended dynamically, loaded as plugins, or stored behind a stable runtime interface.

</details>

---

### Q7 - Overload Pattern

What does this helper enable?

```cpp
template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;
```

<details>
<summary>Answer</summary>

It combines multiple lambdas into one visitor object. Each lambda contributes an `operator()`. This is especially useful with `std::visit`.

</details>

---

### Q8 - `if constexpr`

Why does `if constexpr` help in generic code?

<details>
<summary>Answer</summary>

The condition is evaluated at compile time, and the branch not selected is discarded for the current instantiation. This allows type-dependent logic without requiring every branch to compile for every type.

</details>

---

### Q9 - Fold Expressions

What does this return?

```cpp
template <typename... Checks>
bool all(Checks... checks) {
    return (checks && ...);
}
```

<details>
<summary>Answer</summary>

It returns the logical AND of every argument in the parameter pack. For example, `all(true, true, false)` returns `false`.

For an empty pack, this unary fold over `&&` has identity `true`.

</details>

---

### Q10 - Config Parsing

Why should configuration strings be parsed at the boundary rather than passed deep into robotics subsystems?

<details>
<summary>Answer</summary>

Typed subsystems can enforce invariants early and avoid repeated string parsing. Passing strings deep into the system spreads error handling everywhere and allows invalid values to survive too long.

Parse once into types like `double`, `bool`, `std::optional`, or `std::variant`, validate, then pass a typed config object inward.

</details>

---

## Coding Questions

### Q11 - Optional Parser

Complete `parse_double`.

```cpp
#include <optional>
#include <string>

std::optional<double> parse_double(const std::string& text) {
    // TODO
}
```

It should return `std::nullopt` if the full string is not a valid double.

<details>
<summary>Answer</summary>

```cpp
std::optional<double> parse_double(const std::string& text) {
    try {
        std::size_t pos = 0;
        double value = std::stod(text, &pos);
        if (pos != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}
```

</details>

---

### Q12 - Variant Visitor

Complete `measurement_name`.

```cpp
#include <string>
#include <variant>

struct Imu {};
struct Gps {};
struct Lidar {};

using Measurement = std::variant<Imu, Gps, Lidar>;

std::string measurement_name(const Measurement& m) {
    // TODO
}
```

<details>
<summary>Answer</summary>

```cpp
template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;

std::string measurement_name(const Measurement& m) {
    return std::visit(Overload{
        [](const Imu&) { return std::string{"imu"}; },
        [](const Gps&) { return std::string{"gps"}; },
        [](const Lidar&) { return std::string{"lidar"}; }
    }, m);
}
```

</details>

