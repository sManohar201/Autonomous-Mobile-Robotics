# Q1 - Container Choice

When should you prefer `std::deque` over `std::vector`?

<details>
<summary>Answer</summary>

Use `std::deque` when you need efficient insertion/removal at both ends, such as a sliding time window. `std::vector` is better for contiguous storage and random-access numeric processing.

</details>
