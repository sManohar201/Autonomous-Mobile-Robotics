# Q8 - `if constexpr`

Why does `if constexpr` help in generic code?

<details>
<summary>Answer</summary>

The condition is evaluated at compile time, and the branch not selected is discarded for the current instantiation. This allows type-dependent logic without requiring every branch to compile for every type.

</details>
