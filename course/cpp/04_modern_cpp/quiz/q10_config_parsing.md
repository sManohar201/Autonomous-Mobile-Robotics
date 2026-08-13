# Q10 - Config Parsing

Why should configuration strings be parsed at the boundary rather than passed deep into robotics subsystems?

<details>
<summary>Answer</summary>

Typed subsystems can enforce invariants early and avoid repeated string parsing. Passing strings deep into the system spreads error handling everywhere and allows invalid values to survive too long.

Parse once into types like `double`, `bool`, `std::optional`, or `std::variant`, validate, then pass a typed config object inward.

</details>
