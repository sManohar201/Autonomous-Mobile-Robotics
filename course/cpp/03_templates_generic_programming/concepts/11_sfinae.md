# 3.10 SFINAE

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
