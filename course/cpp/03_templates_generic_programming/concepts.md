# Module 3 - Templates & Generic Programming

**Prerequisites:** Module 1 (toolchain, types, references, namespaces), Module 2 (classes, invariants, RAII, operator overloading)
**Goal:** Write reusable robotics code without sacrificing type safety, performance, or readable diagnostics.

---

## 3.1 Why Generic Programming Matters

Robotics code repeats the same shapes everywhere:

- Buffers of `ImuSample`, `LidarScan`, `Pose2d`, `WheelTick`
- Math functions that should work for `float`, `double`, and sometimes fixed-point numeric types
- Coordinate transforms over 2D and 3D vectors
- Drivers that differ by message type but share queueing, timestamp validation, and diagnostics

Copy-pasting the same implementation for each type is brittle. Runtime polymorphism solves some reuse problems, but it introduces indirection and often forces heap allocation. Templates solve a different problem: they let the compiler generate type-specific code from one source pattern.

```cpp
template <typename T>
T clamp(T value, T lo, T hi) {
    if (value < lo) return lo;
    if (hi < value) return hi;
    return value;
}

int main() {
    int pixels = clamp(300, 0, 255);
    double yaw = clamp(4.2, -3.14159, 3.14159);
}
```

The compiler instantiates `clamp<int>` and `clamp<double>` as needed. There is no virtual call and no runtime type check.

**Key idea:** templates are compile-time code generation with type checking.

---

## 3.2 Function Templates

A function template describes a family of functions.

```cpp
template <typename T>
T square(T x) {
    return x * x;
}

double d = square(2.5);  // T = double
int i = square(7);       // T = int
```

Template argument deduction uses function arguments, not the return type.

```cpp
template <typename T>
T make_zero() {
    return T{};
}

// auto x = make_zero();      // error: T cannot be deduced
auto y = make_zero<double>(); // explicit template argument
```

Deduction also preserves important distinctions:

```cpp
template <typename T>
void inspect(T value);       // passing by value drops top-level const and refs

template <typename T>
void inspect_ref(const T& v); // keeps type without copying
```

For robotics, prefer `const T&` for large message-like objects and plain `T` for small arithmetic values.

---

## 3.3 Class Templates

A class template describes a family of types.

```cpp
template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(std::size_t capacity)
        : capacity_{capacity}, data_{new T[capacity]}
    {}

    ~RingBuffer() { delete[] data_; }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

private:
    std::size_t capacity_;
    T* data_;
};
```

`RingBuffer<ImuSample>` and `RingBuffer<double>` are distinct types. A function taking `RingBuffer<double>` cannot accidentally receive `RingBuffer<int>`.

Class templates are the backbone of generic containers, filters, math types, and message adapters.

---

## 3.4 Non-Type Template Parameters

Templates can take values as parameters, not just types.

```cpp
template <typename T, std::size_t N>
class StaticVector {
public:
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }
    constexpr std::size_t size() const { return N; }

private:
    T data_[N]{};
};

StaticVector<double, 3> accel;
```

The size is part of the type. This is useful when dimensions are fixed by physics:

- `Vector<double, 3>` for acceleration
- `Matrix<double, 6, 6>` for EKF covariance
- `RingBuffer<ImuSample, 512>` for a fixed-size embedded queue

Compile-time dimensions prevent entire classes of runtime mismatch bugs.

---

## 3.5 Template Instantiation and Header-Only Code

Templates are usually implemented in headers because the compiler must see the full definition at the point of instantiation.

```cpp
// math.hpp
template <typename T>
T norm2(T x, T y) {
    return x * x + y * y;
}
```

If you put a template definition only in a `.cpp` file, another translation unit cannot instantiate it unless you explicitly instantiate the needed types:

```cpp
template double norm2<double>(double, double);
template float norm2<float>(float, float);
```

Most teaching code and many robotics utility libraries keep templates header-only for simplicity. Production libraries may use explicit instantiation to reduce compile times and binary size.

---

## 3.6 Specialization

Specialization lets you customize a template for a specific type.

```cpp
template <typename T>
struct SensorTraits {
    static constexpr const char* topic = "/unknown";
    static constexpr double default_rate_hz = 10.0;
};

struct ImuSample {};
struct LidarScan {};

template <>
struct SensorTraits<ImuSample> {
    static constexpr const char* topic = "/imu/data";
    static constexpr double default_rate_hz = 100.0;
};

template <>
struct SensorTraits<LidarScan> {
    static constexpr const char* topic = "/front_laser/scan";
    static constexpr double default_rate_hz = 20.0;
};
```

Traits move type-specific facts out of generic algorithms:

```cpp
template <typename Msg>
void print_default_topic() {
    std::cout << SensorTraits<Msg>::topic << "\n";
}
```

This pattern appears in serialization libraries, ROS message adapters, Eigen traits, and numeric type systems.

---

## 3.7 Partial Specialization

Function templates cannot be partially specialized, but class templates can.

```cpp
template <typename T>
struct IsPointer {
    static constexpr bool value = false;
};

template <typename T>
struct IsPointer<T*> {
    static constexpr bool value = true;
};
```

`IsPointer<int>::value` is false. `IsPointer<int*>::value` is true.

Partial specialization is useful when behavior depends on a type shape: pointer, reference, array, optional, vector, or matrix expression.

---

## 3.8 Type Traits

The standard library provides type traits in `<type_traits>`.

```cpp
#include <type_traits>

template <typename T>
void require_arithmetic(T value) {
    static_assert(std::is_arithmetic_v<T>, "T must be numeric");
    (void)value;
}
```

Traits let you ask compile-time questions:

- Is this type arithmetic?
- Is it copy constructible?
- Is it floating point?
- Does it have a virtual destructor?
- Can it be moved without throwing?

Generic robotics code often needs these questions. Example: a fixed-size vector should accept arithmetic scalar types but reject `std::string`.

---

## 3.9 `if constexpr`

`if constexpr` chooses a branch at compile time. The discarded branch does not need to compile for the current type.

```cpp
#include <type_traits>
#include <cmath>

template <typename T>
double magnitude(T x, T y) {
    if constexpr (std::is_integral_v<T>) {
        return std::sqrt(static_cast<double>(x * x + y * y));
    } else {
        return std::sqrt(x * x + y * y);
    }
}
```

This is cleaner than SFINAE for many simple cases.

Use `if constexpr` when one algorithm has a small type-dependent branch. Use specialization when the whole implementation meaningfully differs by type.

---

## 3.10 SFINAE

SFINAE means "substitution failure is not an error." If template substitution fails while checking a candidate overload, the compiler removes that overload instead of producing an immediate error.

```cpp
#include <type_traits>

template <typename T,
          typename = std::enable_if_t<std::is_floating_point_v<T>>>
T reciprocal(T x) {
    return T{1} / x;
}
```

This function only participates in overload resolution for floating-point types.

SFINAE is powerful but hard to read. In modern C++, prefer concepts when available.

---

## 3.11 C++20 Concepts

Concepts name compile-time requirements.

```cpp
#include <concepts>

template <typename T>
concept Scalar = std::integral<T> || std::floating_point<T>;

template <Scalar T>
T saturate(T value, T lo, T hi) {
    if (value < lo) return lo;
    if (hi < value) return hi;
    return value;
}
```

Concepts improve diagnostics. Instead of pages of template errors, the compiler can say the type does not satisfy `Scalar`.

You can also define structural requirements:

```cpp
template <typename T>
concept HasTimestamp = requires(const T& msg) {
    { msg.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    return a.timestamp_ns > b.timestamp_ns;
}
```

This is a strong fit for robotics message-processing utilities.

---

## 3.12 CRTP

CRTP is the Curiously Recurring Template Pattern: a base class template takes the derived class as a template parameter.

```cpp
template <typename Derived>
class Printable {
public:
    void print() const {
        static_cast<const Derived&>(*this).print_impl();
    }
};

class Pose2d : public Printable<Pose2d> {
public:
    void print_impl() const {
        std::cout << "Pose2d\n";
    }
};
```

This gives compile-time polymorphism. There is no virtual table and no dynamic dispatch.

Use CRTP when:

- The set of derived types is known at compile time
- You need zero-overhead static dispatch
- You want to share interface glue without runtime polymorphism

Do not use CRTP when you need heterogeneous runtime collections like `std::vector<std::unique_ptr<SensorBase>>`. That is Module 2 runtime polymorphism territory.

---

## 3.13 Generic Physical Units

Templates can make invalid physics unrepresentable.

```cpp
template <typename Tag>
class Quantity {
public:
    explicit constexpr Quantity(double value) : value_{value} {}
    constexpr double value() const { return value_; }

private:
    double value_;
};

struct MeterTag {};
struct SecondTag {};

using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;
```

`Meters` and `Seconds` both store `double`, but they are different types. You cannot accidentally pass seconds where meters are expected unless you explicitly convert.

This pattern scales into dimensional analysis libraries, safer configuration parsers, and strongly typed navigation APIs.

---

## 3.14 Template Error Discipline

Bad generic code produces unreadable compiler diagnostics. Good generic code fails near the call site with a clear reason.

Use these tools:

- `static_assert` for invariants that must always hold
- Concepts for named requirements
- Traits for type-specific metadata
- Small templates with clear responsibilities
- Tests that instantiate templates with more than one type

```cpp
template <typename T>
class Covariance {
    static_assert(std::is_floating_point_v<T>,
                  "Covariance<T> requires a floating-point scalar");
};
```

**Key takeaway:** generic programming is not about being clever. It is about capturing repeated structure once and making invalid uses fail at compile time.

