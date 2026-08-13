# 4.8 `std::optional`

`std::optional<T>` represents either a `T` or no value.

```cpp
std::optional<double> parse_double(const std::string& text);
```

This is better than returning magic sentinel values like `-1.0` or `NaN` when absence is expected and not exceptional.

```cpp
auto maybe_rate = parse_double(config["rate_hz"]);
if (!maybe_rate) {
    std::cout << "rate missing or invalid\n";
} else {
    std::cout << "rate=" << *maybe_rate << "\n";
}
```

Use `optional` for recoverable absence. Use exceptions or error objects for failures that require explanation.
