# Q5 - `reserve`

What problem does `vector.reserve(n)` solve?

<details>
<summary>Answer</summary>

It preallocates capacity so repeated `push_back` calls avoid multiple reallocations and element moves/copies.

</details>
