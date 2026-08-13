# Q3 - FSM

Why prefer `enum class` over plain `enum` for states?

<details><summary>Answer</summary>

`enum class` is scoped and does not implicitly convert to integers. This reduces accidental comparisons and namespace pollution.

</details>
