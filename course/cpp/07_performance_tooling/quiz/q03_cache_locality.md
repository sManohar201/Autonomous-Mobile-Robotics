# Q3 - Cache Locality

Why is `std::vector` usually better than `std::list` for point clouds?

<details><summary>Answer</summary>

Point clouds are traversed sequentially. `std::vector` stores elements contiguously, which is cache-friendly. `std::list` scatters nodes across memory and adds pointer chasing.

</details>
