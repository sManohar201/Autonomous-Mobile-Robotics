# 4.12 Fold Expressions

Fold expressions reduce a parameter pack.

```cpp
template <typename... Values>
auto sum(Values... values) {
    return (values + ...);
}
```

They are useful for compact validation:

```cpp
template <typename... Checks>
bool all(Checks... checks) {
    return (checks && ...);
}
```

For a robotics configuration object:

```cpp
bool ok = all(rate_hz > 0.0, timeout_ms > 0, frame_id != "");
```

Fold expressions are powerful, but do not use them to hide complex logic.
