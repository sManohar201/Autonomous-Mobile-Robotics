# **Q2.** You put the definition of a free function in a header and include it in three `.cpp` files. What happens at link time and why?

<details>
<summary>Answer</summary>

The linker sees three copies of the same symbol (one from each `.o` file) and raises a **multiple definition** error. Each `.cpp` that includes the header gets its own copy after preprocessing. The linker requires exactly one definition per symbol.

Fix: move the definition to a `.cpp` file, or mark the function `inline`.

</details>
