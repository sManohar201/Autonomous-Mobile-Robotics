# Q1 - Observer Pattern

What is the primary risk when observers are callbacks?

<details><summary>Answer</summary>

Lifetime and reentrancy. A callback may refer to destroyed objects, call back into the publisher, block, or throw. Avoid holding locks while invoking observers.

</details>
