# Q5 - `std::optional`

When is `std::optional<T>` a better return type than returning `T` with a sentinel value?

<details>
<summary>Answer</summary>

Use `optional` when absence is expected and recoverable. It makes "no value" explicit in the type system.

Sentinel values are fragile because they can overlap with valid values or be forgotten by callers.

</details>
