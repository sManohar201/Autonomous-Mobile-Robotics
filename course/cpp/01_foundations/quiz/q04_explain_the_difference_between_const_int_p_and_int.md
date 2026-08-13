# **Q4.** Explain the difference between `const int* p` and `int* const p`.

<details>
<summary>Answer</summary>

`const int* p` — pointer to const int. Cannot modify `*p`. Can reassign `p` to point elsewhere.

`int* const p` — const pointer to int. Can modify `*p`. Cannot reassign `p`.

Read right-to-left: `const int* p` → "p is a pointer to const int".

</details>
