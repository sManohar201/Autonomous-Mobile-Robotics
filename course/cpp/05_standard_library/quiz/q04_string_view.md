# Q4 - `string_view`

What is the main lifetime risk of `std::string_view`?

<details>
<summary>Answer</summary>

It does not own the characters. If the referenced string is destroyed or modified incompatibly, the view dangles. Use it for parameters, not long-term storage unless lifetime is guaranteed.

</details>
