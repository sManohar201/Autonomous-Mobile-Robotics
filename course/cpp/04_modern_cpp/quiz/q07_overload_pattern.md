# Q7 - Overload Pattern

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
