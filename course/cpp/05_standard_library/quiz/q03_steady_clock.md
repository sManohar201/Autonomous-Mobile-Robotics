# Q3 - `steady_clock`

Why is `std::chrono::steady_clock` better than system time for measuring elapsed duration?

<details>
<summary>Answer</summary>

`steady_clock` is monotonic. System time can move backward or forward due to NTP/manual changes, which can break timeout and duration calculations.

</details>
