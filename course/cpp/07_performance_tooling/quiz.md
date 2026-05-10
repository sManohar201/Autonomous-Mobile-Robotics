# Module 7 Quiz - Performance & Tooling

## Q1 - Debug vs Release

Why should benchmarks use Release builds?

<details><summary>Answer</summary>

Debug builds disable or reduce optimization and include extra debug information. They can be orders of magnitude slower and do not represent production performance.

</details>

---

## Q2 - `reserve`

How can `reserve` reduce latency jitter?

<details><summary>Answer</summary>

It preallocates vector capacity and avoids repeated reallocations during `push_back`, reducing unpredictable allocation and copy/move costs.

</details>

---

## Q3 - Cache Locality

Why is `std::vector` usually better than `std::list` for point clouds?

<details><summary>Answer</summary>

Point clouds are traversed sequentially. `std::vector` stores elements contiguously, which is cache-friendly. `std::list` scatters nodes across memory and adds pointer chasing.

</details>

---

## Q4 - Sanitizers

Which sanitizer would you use for data races?

<details><summary>Answer</summary>

ThreadSanitizer.

</details>

---

## Q5 - Measurement

Why is a single timing run weak evidence?

<details><summary>Answer</summary>

Timing varies due to scheduler noise, CPU frequency changes, cache state, and background processes. Use repeated runs and compare medians or distributions.

</details>

