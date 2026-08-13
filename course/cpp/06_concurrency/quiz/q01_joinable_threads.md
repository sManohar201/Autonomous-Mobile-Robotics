# Q1 - Joinable Threads

What happens if a joinable `std::thread` is destroyed?

<details><summary>Answer</summary>

The program calls `std::terminate`. You must `join()` or `detach()` before destruction. In most robotics code, prefer `join()` for controlled shutdown.

</details>
