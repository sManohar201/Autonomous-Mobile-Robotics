# Module 3 Quiz - Templates & Generic Programming

12 questions covering sections 3.1-3.14. Work through each question before expanding the answer.

---

## Conceptual Questions

### Q1 - Function Template Deduction

Given:

```cpp
template <typename T>
T max_value(T a, T b) {
    return a < b ? b : a;
}
```

Why does `max_value(1, 2.5)` fail to compile, and what are two clean ways to call it?

<details>
<summary>Answer</summary>

Template argument deduction tries to infer one type `T` for both arguments. The first argument suggests `T = int`; the second suggests `T = double`. The compiler does not choose a common type during template deduction for this function, so deduction fails.

Two clean calls:

```cpp
auto a = max_value<double>(1, 2.5);     // explicitly choose T
auto b = max_value(1.0, 2.5);           // make both arguments double
```

You could also design a two-parameter template using `std::common_type_t`, but that is a different interface.

</details>

---

### Q2 - Templates vs Runtime Polymorphism

When would you prefer a template-based sensor utility over a virtual base class? When would a virtual base class be the better choice?

<details>
<summary>Answer</summary>

Prefer templates when the type is known at compile time and you want zero-overhead, type-specific code: fixed-size math, typed buffers, compile-time traits, or algorithms over message structs.

Prefer runtime polymorphism when you need to store different concrete types behind one interface at runtime, such as:

```cpp
std::vector<std::unique_ptr<SensorPlugin>>
```

Templates give compile-time polymorphism. Virtual functions give runtime polymorphism. They solve different problems.

</details>

---

### Q3 - Header-Only Templates

Why are template definitions usually placed in header files?

<details>
<summary>Answer</summary>

The compiler must see the full template definition at the point where it instantiates a concrete function or class. If another `.cpp` file only sees a declaration, it does not have enough information to generate `Template<T>` for the requested `T`.

Putting template definitions in headers makes the definition visible to every translation unit. The alternative is explicit instantiation for every supported type in a `.cpp` file.

</details>

---

### Q4 - Non-Type Template Parameters

Why is `Vector<double, 3>` safer than a vector class that stores its dimension as a runtime integer?

<details>
<summary>Answer</summary>

The dimension is part of the type. A function that expects `Vector<double, 3>` cannot accidentally receive `Vector<double, 2>`. This catches dimension mismatch at compile time instead of producing a runtime error or, worse, a silent math bug.

This is useful for robotics math: 3D acceleration, 6D state vectors, 6x6 covariance matrices, and fixed-capacity embedded buffers.

</details>

---

### Q5 - Traits

What is a traits class, and why is it useful in generic robotics code?

<details>
<summary>Answer</summary>

A traits class stores type-specific metadata or behavior outside the type itself.

```cpp
template <typename Msg>
struct SensorTraits;
```

Specializations can define facts such as default topic, expected rate, frame id, or whether a message has covariance. Generic code can then use `SensorTraits<Msg>` without hard-coding message-specific logic into the algorithm.

</details>

---

### Q6 - Partial Specialization

Can function templates be partially specialized? What should you use instead?

<details>
<summary>Answer</summary>

Function templates cannot be partially specialized. Use overloads, constraints/concepts, or helper class templates that can be partially specialized.

Class templates can be partially specialized:

```cpp
template <typename T>
struct IsPointer { static constexpr bool value = false; };

template <typename T>
struct IsPointer<T*> { static constexpr bool value = true; };
```

</details>

---

### Q7 - `if constexpr`

Why can `if constexpr` compile code where a normal `if` would fail?

<details>
<summary>Answer</summary>

`if constexpr` evaluates the condition at compile time. The branch not selected is discarded before type checking for the instantiated template. A normal `if` requires both branches to compile, even if one branch will never execute at runtime.

This lets one template contain small type-dependent branches.

</details>

---

### Q8 - SFINAE vs Concepts

What problem did SFINAE solve, and why are C++20 concepts usually clearer?

<details>
<summary>Answer</summary>

SFINAE lets invalid template substitutions remove an overload from consideration instead of producing a hard compiler error. It enabled constrained templates before C++20.

Concepts express requirements directly and name them:

```cpp
template <std::floating_point T>
T reciprocal(T x);
```

Diagnostics are usually shorter and more directly tied to the failed requirement.

</details>

---

### Q9 - CRTP

What does CRTP provide, and what is its main limitation compared with virtual functions?

<details>
<summary>Answer</summary>

CRTP provides compile-time polymorphism. The base class can call methods on the derived type using `static_cast<Derived&>(*this)`, avoiding virtual dispatch.

Its main limitation is that it does not provide runtime polymorphism. You cannot use CRTP alone to store unrelated derived types in one homogeneous runtime collection through a base pointer.

</details>

---

### Q10 - Compile-Time Units

Why is this valuable?

```cpp
using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;
```

<details>
<summary>Answer</summary>

Both types may store a `double`, but they are distinct at compile time. A function expecting `Meters` cannot accidentally receive `Seconds`. This prevents unit bugs without runtime overhead.

The larger design goal is to make invalid physical operations unrepresentable or at least explicit.

</details>

---

## Coding Questions

### Q11 - Constrained Timestamp Utility

Complete `is_newer` so it only accepts types with a `timestamp_ns` member convertible to `std::uint64_t`.

```cpp
#include <concepts>
#include <cstdint>

template <typename T>
concept HasTimestamp = /* TODO */;

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    /* TODO */
}
```

<details>
<summary>Answer</summary>

```cpp
#include <concepts>
#include <cstdint>

template <typename T>
concept HasTimestamp = requires(const T& msg) {
    { msg.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    return a.timestamp_ns > b.timestamp_ns;
}
```

</details>

---

### Q12 - Fixed-Size Vector Dot Product

Complete the generic dot product.

```cpp
template <typename T, std::size_t N>
class Vector {
public:
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }

private:
    T data_[N]{};
};

template <typename T, std::size_t N>
T dot(const Vector<T, N>& a, const Vector<T, N>& b) {
    /* TODO */
}
```

<details>
<summary>Answer</summary>

```cpp
template <typename T, std::size_t N>
T dot(const Vector<T, N>& a, const Vector<T, N>& b) {
    T sum{};
    for (std::size_t i = 0; i < N; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}
```

The dimension is part of the type, so `dot(Vector<double, 2>, Vector<double, 3>)` does not match this overload.

</details>

