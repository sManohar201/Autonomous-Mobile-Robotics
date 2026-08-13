# 5.5 Erase-Remove

`std::remove_if` rearranges elements and returns the new logical end. It does not shrink the container. Pair it with `erase`:

```cpp
values.erase(std::remove_if(values.begin(), values.end(), pred), values.end());
```

This pattern appears often when dropping invalid sensor readings.
