# Q11 - Constrained Timestamp Utility

## Coding Questions

Complete `is_newer` so it only accepts types with a `timestamp_ns` member convertible to `std::uint64_t`.

```cpp
#include <concepts>
#include <cstdint>

template <typename T>
concept HasTimestamp = /* TODO */;

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    /* TODO */
}
```

<details>
<summary>Answer</summary>

```cpp
#include <concepts>
#include <cstdint>

template <typename T>
concept HasTimestamp = requires(const T& msg) {
    { msg.timestamp_ns } -> std::convertible_to<std::uint64_t>;
};

template <HasTimestamp Msg>
bool is_newer(const Msg& a, const Msg& b) {
    return a.timestamp_ns > b.timestamp_ns;
}
```

</details>
