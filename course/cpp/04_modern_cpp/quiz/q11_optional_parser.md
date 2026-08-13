# Q11 - Optional Parser

## Coding Questions

Complete `parse_double`.

```cpp
#include <optional>
#include <string>

std::optional<double> parse_double(const std::string& text) {
    // TODO
}
```

It should return `std::nullopt` if the full string is not a valid double.

<details>
<summary>Answer</summary>

```cpp
std::optional<double> parse_double(const std::string& text) {
    try {
        std::size_t pos = 0;
        double value = std::stod(text, &pos);
        if (pos != text.size()) {
            return std::nullopt;
        }
        return value;
    } catch (...) {
        return std::nullopt;
    }
}
```

</details>
