# 3.11 C++20 Concepts

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
