# 3.2 Function Templates

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
