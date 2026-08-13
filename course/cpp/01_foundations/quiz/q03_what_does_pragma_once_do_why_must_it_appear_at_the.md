# **Q3.** What does `#pragma once` do? Why must it appear at the top of a header?

<details>
<summary>Answer</summary>

It tells the compiler to include this file at most once per translation unit, preventing redefinition errors from repeated inclusion (direct or transitive). It must appear before any code so it takes effect before any declarations are processed.

</details>
