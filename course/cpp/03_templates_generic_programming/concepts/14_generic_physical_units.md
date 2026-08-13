# 3.13 Generic Physical Units

Templates can make invalid physics unrepresentable.

```cpp
template <typename Tag>
class Quantity {
public:
    explicit constexpr Quantity(double value) : value_{value} {}
    constexpr double value() const { return value_; }

private:
    double value_;
};

struct MeterTag {};
struct SecondTag {};

using Meters = Quantity<MeterTag>;
using Seconds = Quantity<SecondTag>;
```

`Meters` and `Seconds` both store `double`, but they are different types. You cannot accidentally pass seconds where meters are expected unless you explicitly convert.

This pattern scales into dimensional analysis libraries, safer configuration parsers, and strongly typed navigation APIs.
