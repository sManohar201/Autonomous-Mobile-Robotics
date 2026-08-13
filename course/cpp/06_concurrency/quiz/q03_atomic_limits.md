# Q3 - Atomic Limits

Why is `std::atomic<bool>` not enough to protect a queue?

<details><summary>Answer</summary>

The queue has compound state: storage, size, head/tail, and invariants across those fields. A mutex is needed to protect those multi-step updates.

</details>
