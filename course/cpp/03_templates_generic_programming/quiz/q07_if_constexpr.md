# Q7 - `if constexpr`

Why can `if constexpr` compile code where a normal `if` would fail?

<details>
<summary>Answer</summary>

`if constexpr` evaluates the condition at compile time. The branch not selected is discarded before type checking for the instantiated template. A normal `if` requires both branches to compile, even if one branch will never execute at runtime.

This lets one template contain small type-dependent branches.

</details>
