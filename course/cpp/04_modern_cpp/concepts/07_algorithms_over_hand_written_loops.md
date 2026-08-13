# 4.6 Algorithms Over Hand-Written Loops

The standard algorithms make intent explicit.

```cpp
auto valid = std::all_of(samples.begin(), samples.end(),
                         [](const auto& s) { return s.range_m >= 0.0; });
```

Common algorithms:

- `std::find_if`
- `std::count_if`
- `std::transform`
- `std::accumulate`
- `std::all_of`, `std::any_of`, `std::none_of`
- `std::sort`

Use algorithms when they clarify intent. Use explicit loops when the control flow is genuinely more readable.
