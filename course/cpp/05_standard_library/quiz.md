# Module 5 Quiz - Standard Library

## Q1 - Container Choice

When should you prefer `std::deque` over `std::vector`?

<details>
<summary>Answer</summary>

Use `std::deque` when you need efficient insertion/removal at both ends, such as a sliding time window. `std::vector` is better for contiguous storage and random-access numeric processing.

</details>

---

## Q2 - Erase-Remove

Why is `std::remove_if` alone not enough to delete elements from a vector?

<details>
<summary>Answer</summary>

`remove_if` only moves kept elements toward the front and returns the new logical end. The vector size is unchanged. You must call `erase(new_end, end)` to shrink it.

</details>

---

## Q3 - `steady_clock`

Why is `std::chrono::steady_clock` better than system time for measuring elapsed duration?

<details>
<summary>Answer</summary>

`steady_clock` is monotonic. System time can move backward or forward due to NTP/manual changes, which can break timeout and duration calculations.

</details>

---

## Q4 - `string_view`

What is the main lifetime risk of `std::string_view`?

<details>
<summary>Answer</summary>

It does not own the characters. If the referenced string is destroyed or modified incompatibly, the view dangles. Use it for parameters, not long-term storage unless lifetime is guaranteed.

</details>

---

## Q5 - `reserve`

What problem does `vector.reserve(n)` solve?

<details>
<summary>Answer</summary>

It preallocates capacity so repeated `push_back` calls avoid multiple reallocations and element moves/copies.

</details>

