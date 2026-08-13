# Q1 - `auto` and References

## Conceptual Questions

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
