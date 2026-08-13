# 4.2 `auto`

`auto` asks the compiler to deduce a variable type from its initializer.

```cpp
auto rate_hz = 50.0;     // double
auto count = 10;         // int
auto name = "imu";       // const char*
```

Use `auto` when the initializer already makes the type obvious or when the exact type is verbose:

```cpp
auto it = measurements.find(sensor_id);
auto lock = std::scoped_lock{mutex};
```

Do not use `auto` when it hides important semantics:

```cpp
auto timeout = 1000; // milliseconds? seconds? message count?
```

In that case, write the type or use a named duration:

```cpp
std::chrono::milliseconds timeout{1000};
```

Plain `auto` drops references and top-level `const`.

```cpp
const int value = 42;
auto a = value;        // int
const auto b = value;  // const int
auto& c = value;       // const int&
```

Use `const auto&` when iterating over large messages.
