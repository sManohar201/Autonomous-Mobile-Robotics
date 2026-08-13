# 3.14 Template Error Discipline

Bad generic code produces unreadable compiler diagnostics. Good generic code fails near the call site with a clear reason.

Use these tools:

- `static_assert` for invariants that must always hold
- Concepts for named requirements
- Traits for type-specific metadata
- Small templates with clear responsibilities
- Tests that instantiate templates with more than one type

```cpp
template <typename T>
class Covariance {
    static_assert(std::is_floating_point_v<T>,
                  "Covariance<T> requires a floating-point scalar");
};
```

**Key takeaway:** generic programming is not about being clever. It is about capturing repeated structure once and making invalid uses fail at compile time.
