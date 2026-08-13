# 4.4 Lambdas

A lambda creates a small function object.

```cpp
auto is_recent = [now](const Measurement& m) {
    return now - m.timestamp_ns < 100000000;
};
```

Capture deliberately:

- `[x]` captures `x` by value
- `[&x]` captures `x` by reference
- `[=]` captures used variables by value
- `[&]` captures used variables by reference

Avoid broad captures in long-lived callbacks. In robotics, callbacks may outlive the scope you had in mind.

```cpp
auto callback = [this](const ImuMsg& msg) {
    filter_.update(msg);
};
```

This is only safe if `this` is guaranteed to outlive the callback registration.
