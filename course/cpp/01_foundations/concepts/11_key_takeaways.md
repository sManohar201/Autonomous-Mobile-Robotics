# Key Takeaways

1. **Pipeline**: preprocessor → compiler → linker. "Undefined reference" = linker error.
2. **Headers = declarations, source = definitions.** Definitions in headers cause multiple-definition errors.
3. **`#pragma once`** in every header.
4. **Use `double`** for physics. Use fixed-width types (`int64_t`, `uint8_t`) when size matters.
5. **`const` is correctness**, not style. Mark parameters, member functions, and variables `const` wherever possible.
6. **`const&` for function parameters** — no copy.
7. **References can't be null.** Use pointers when null is valid.
8. **Never `using namespace X;` in a header.**
9. **CMake `PUBLIC`/`PRIVATE`** — get this right or consumers won't find your headers.

---

→ [Exercises](../exercises/) → [Quiz](../quiz/)
