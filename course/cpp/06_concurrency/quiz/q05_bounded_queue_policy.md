# Q5 - Bounded Queue Policy

Why should a bounded queue define its overload behavior explicitly?

<details><summary>Answer</summary>

Robotics systems need predictable behavior under load. The queue should clearly block, reject new items, drop oldest, or drop newest rather than growing unbounded and failing later.

</details>
