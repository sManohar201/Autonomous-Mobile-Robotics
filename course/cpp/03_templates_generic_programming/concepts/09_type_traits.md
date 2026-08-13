# 3.8 Type Traits

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
