# Q1 - Function Template Deduction

## Conceptual Questions

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
