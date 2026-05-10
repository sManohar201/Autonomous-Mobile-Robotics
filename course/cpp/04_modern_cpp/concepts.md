# Module 4 - Modern C++

**Prerequisites:** Module 1 (types, references, namespaces), Module 2 (classes and RAII), Module 3 (templates, concepts, compile-time constraints)
**Goal:** Use modern C++ features to make robotics code clearer, safer, and more expressive without hiding control flow or ownership.

---

## 4.1 What "Modern C++" Means

Modern C++ is not a list of syntax tricks. It is a shift toward:

- Expressing intent directly
- Reducing manual type noise
- Avoiding invalid sentinel values
- Making ownership and absence explicit
- Moving decisions from runtime to compile time when practical

Robotics code benefits from this because failures often happen at subsystem boundaries: configuration files, sensor messages, state machines, and callback pipelines. Modern C++ gives you stronger vocabulary types for those boundaries.

---

## 4.2 `auto`

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

---

## 4.3 Structured Bindings

Structured bindings unpack tuple-like or aggregate values.

```cpp
struct Pose2d {
    double x;
    double y;
    double theta;
};

Pose2d pose{1.0, 2.0, 0.5};
auto [x, y, theta] = pose;
```

They are especially useful with maps:

```cpp
for (const auto& [name, rate_hz] : sensor_rates) {
    std::cout << name << " runs at " << rate_hz << " Hz\n";
}
```

Be intentional about copies:

```cpp
auto [k, v] = *it;        // copies key and value
auto& [rk, rv] = *it;     // references pair members
const auto& [ck, cv] = *it;
```

---

## 4.4 Lambdas

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

---

## 4.5 Generic Lambdas

Generic lambdas use `auto` parameters.

```cpp
auto print_timestamp = [](const auto& msg) {
    std::cout << msg.timestamp_ns << "\n";
};
```

This is shorthand for a templated function object. It is useful for small local algorithms, not for public interfaces where named concepts and clearer diagnostics matter.

---

## 4.6 Algorithms Over Hand-Written Loops

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

---

## 4.7 `std::function`

`std::function` stores any callable with a matching signature.

```cpp
std::function<void(double)> update_callback;

update_callback = [](double dt) {
    std::cout << "dt=" << dt << "\n";
};
```

It is useful for callback registration and plugin-like seams:

```cpp
class Timer {
public:
    using Callback = std::function<void(double)>;
    explicit Timer(Callback cb) : callback_{std::move(cb)} {}

    void tick(double dt) { callback_(dt); }

private:
    Callback callback_;
};
```

Tradeoff: `std::function` may allocate and performs type-erased dispatch. For hot inner loops, prefer templates or direct function objects.

---

## 4.8 `std::optional`

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

---

## 4.9 `std::variant`

`std::variant` stores one value from a fixed set of alternatives.

```cpp
struct ImuMeasurement { double ax, ay, az; };
struct GpsMeasurement { double lat, lon, alt; };

using Measurement = std::variant<ImuMeasurement, GpsMeasurement>;
```

Visit a variant with `std::visit`:

```cpp
std::visit([](const auto& msg) {
    std::cout << "received measurement\n";
}, measurement);
```

Use variants when all alternatives are known and closed. Use runtime polymorphism when new alternatives must be added through plugins or dynamic loading.

---

## 4.10 Overload Pattern for Variant Visitors

The overload pattern combines multiple lambdas into one visitor.

```cpp
template <class... Ts>
struct Overload : Ts... {
    using Ts::operator()...;
};

template <class... Ts>
Overload(Ts...) -> Overload<Ts...>;
```

Usage:

```cpp
std::visit(Overload{
    [](const ImuMeasurement& imu) { handle_imu(imu); },
    [](const GpsMeasurement& gps) { handle_gps(gps); }
}, measurement);
```

This is a compact way to write type-specific logic while keeping the set of cases explicit.

---

## 4.11 `if constexpr`

`if constexpr` was introduced in Module 3 for templates. In modern C++, it often replaces complicated overload sets for small compile-time decisions.

```cpp
template <typename Msg>
std::string frame_id_for() {
    if constexpr (std::is_same_v<Msg, ImuMeasurement>) {
        return "imu_link";
    } else if constexpr (std::is_same_v<Msg, GpsMeasurement>) {
        return "gps_link";
    } else {
        return "base_link";
    }
}
```

The discarded branches are not instantiated for the current type.

---

## 4.12 Fold Expressions

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

---

## 4.13 Config Parsing with Vocabulary Types

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

---

## 4.14 Small State Machines

State machines appear in robot behavior, sensor drivers, lifecycle nodes, and recovery logic.

```cpp
enum class State {
    Idle,
    Armed,
    Running,
    Fault
};

enum class Event {
    Configure,
    Start,
    Stop,
    Error
};
```

A transition function makes behavior explicit:

```cpp
constexpr State transition(State s, Event e) {
    switch (s) {
    case State::Idle:
        return e == Event::Configure ? State::Armed : s;
    case State::Armed:
        return e == Event::Start ? State::Running : s;
    case State::Running:
        return e == Event::Stop ? State::Armed : s;
    case State::Fault:
        return State::Fault;
    }
    return State::Fault;
}
```

If the transition function is `constexpr`, you can test behavior at compile time:

```cpp
static_assert(transition(State::Idle, Event::Configure) == State::Armed);
```

For large behavior trees or lifecycle graphs, use dedicated architecture. For small deterministic components, a simple typed state machine is often clearer and safer.

---

## 4.15 Practical Rules

- Use `auto` when it improves readability, not to avoid thinking about types.
- Prefer `const auto&` for large objects in loops.
- Use structured bindings for tuple-like return values and map iteration.
- Use lambdas for local behavior; use named functions when behavior is reused or tested separately.
- Use `optional` instead of sentinel values for expected absence.
- Use `variant` when the set of alternatives is closed and known.
- Use `std::function` at callback boundaries, not in tight numeric loops.
- Use `constexpr` and `static_assert` for small rules that should be proven at compile time.

**Key takeaway:** Modern C++ is useful when it makes constraints and intent more visible. If it makes the code clever but harder to audit, it is the wrong tool.

