# Q2 - Erase-Remove

Why is `std::remove_if` alone not enough to delete elements from a vector?

<details>
<summary>Answer</summary>

`remove_if` only moves kept elements toward the front and returns the new logical end. The vector size is unchanged. You must call `erase(new_end, end)` to shrink it.

</details>
