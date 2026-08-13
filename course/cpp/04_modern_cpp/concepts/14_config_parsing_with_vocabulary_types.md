# 4.13 Config Parsing with Vocabulary Types

Configuration is a common source of robotics bugs. Modern C++ gives better tools:

- `std::optional<T>` for missing or invalid values
- `std::variant` for typed config values
- lambdas for local parsers
- structured bindings for key/value iteration

```cpp
using ConfigValue = std::variant<int, double, std::string, bool>;

struct ControllerConfig {
    double rate_hz;
    std::string frame_id;
    bool publish_debug;
};
```

A parser should separate lexing text into keys and values, converting values into typed fields, and validating invariants.

Do not let every subsystem parse strings independently. Parse once at the boundary and pass typed configuration inward.
