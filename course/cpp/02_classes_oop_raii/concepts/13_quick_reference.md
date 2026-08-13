# Quick Reference

| Concept | Rule of thumb |
|---|---|
| struct vs class | struct = data, class = invariants |
| explicit | Always on single-arg constructors unless implicit conversion is intended |
| MIL order | Matches declaration order, not MIL order |
| const member fn | Mark anything that doesn't mutate state as const |
| Rule of Three | Destructor + copy ctor + copy assign — define all or none |
| Rule of Five | + move ctor + move assign; mark both noexcept |
| RAII | Resource in constructor, release in destructor |
| Smart pointers | unique_ptr by default; shared_ptr only for genuinely shared ownership |
| Virtual destructor | Required in any base class used polymorphically |
| override | Always use on overriding functions |
| Slicing | Store polymorphic objects through pointers/references, not values |
| static members | Defined in .cpp; static fn has no this |
