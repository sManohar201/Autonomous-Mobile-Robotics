# Module 8 Quiz - Robotics Patterns

## Q1 - Observer Pattern

What is the primary risk when observers are callbacks?

<details><summary>Answer</summary>

Lifetime and reentrancy. A callback may refer to destroyed objects, call back into the publisher, block, or throw. Avoid holding locks while invoking observers.

</details>

---

## Q2 - Factory

Why use a factory for localization filters?

<details><summary>Answer</summary>

It centralizes construction and lets the rest of the system depend on an interface rather than concrete filter classes. Configuration can select implementations at runtime.

</details>

---

## Q3 - FSM

Why prefer `enum class` over plain `enum` for states?

<details><summary>Answer</summary>

`enum class` is scoped and does not implicitly convert to integers. This reduces accidental comparisons and namespace pollution.

</details>

---

## Q4 - Component Storage

What problem does entity-component style solve?

<details><summary>Answer</summary>

It allows optional sets of data to be attached to many entities without forcing a deep inheritance hierarchy.

</details>

---

## Q5 - Pattern Discipline

When should you not introduce a pattern?

<details><summary>Answer</summary>

When the simpler design is clearer and the expected extension pressure is speculative. Patterns should reduce complexity under real constraints, not add abstraction for its own sake.

</details>

