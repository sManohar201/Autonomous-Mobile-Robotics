# 3.9 `if constexpr`

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
