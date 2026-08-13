# 4.5 Generic Lambdas

Generic lambdas use `auto` parameters.

```cpp
auto print_timestamp = [](const auto& msg) {
    std::cout << msg.timestamp_ns << "\n";
};
```

This is shorthand for a templated function object. It is useful for small local algorithms, not for public interfaces where named concepts and clearer diagnostics matter.
