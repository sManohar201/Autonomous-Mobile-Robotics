# Q3 - Header-Only Templates

Why are template definitions usually placed in header files?

<details>
<summary>Answer</summary>

The compiler must see the full template definition at the point where it instantiates a concrete function or class. If another `.cpp` file only sees a declaration, it does not have enough information to generate `Template<T>` for the requested `T`.

Putting template definitions in headers makes the definition visible to every translation unit. The alternative is explicit instantiation for every supported type in a `.cpp` file.

</details>
