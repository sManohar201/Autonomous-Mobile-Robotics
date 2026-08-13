# **Q1.** What is the difference between a compiler error and a linker error? Give one example of each.

## Part A — Written Questions

<details>
<summary>Answer</summary>

**Compiler error** — caught during compilation of a single `.cpp` file. The compiler checks syntax and types but knows nothing about other translation units.
Example: calling a function with wrong argument types, using an undeclared variable.

**Linker error** — caught when combining `.o` files. The linker cannot find a definition that was declared but never implemented.
Example: `undefined reference to 'compute_hypotenuse(double, double)'` — the declaration was in a header but no `.o` file contained the definition.

</details>
