# 4.15 Practical Rules

- Use `auto` when it improves readability, not to avoid thinking about types.
- Prefer `const auto&` for large objects in loops.
- Use structured bindings for tuple-like return values and map iteration.
- Use lambdas for local behavior; use named functions when behavior is reused or tested separately.
- Use `optional` instead of sentinel values for expected absence.
- Use `variant` when the set of alternatives is closed and known.
- Use `std::function` at callback boundaries, not in tight numeric loops.
- Use `constexpr` and `static_assert` for small rules that should be proven at compile time.

**Key takeaway:** Modern C++ is useful when it makes constraints and intent more visible. If it makes the code clever but harder to audit, it is the wrong tool.
