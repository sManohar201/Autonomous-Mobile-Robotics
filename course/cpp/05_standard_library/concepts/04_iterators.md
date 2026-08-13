# 5.3 Iterators

Iterators generalize positions in a sequence. Algorithms operate on iterator ranges:

```cpp
std::sort(values.begin(), values.end());
auto it = std::find(values.begin(), values.end(), 42);
```

The half-open range `[begin, end)` is standard: begin is included, end is one past the last element.
