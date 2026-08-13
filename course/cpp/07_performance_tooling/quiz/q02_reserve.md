# Q2 - `reserve`

How can `reserve` reduce latency jitter?

<details><summary>Answer</summary>

It preallocates vector capacity and avoids repeated reallocations during `push_back`, reducing unpredictable allocation and copy/move costs.

</details>
