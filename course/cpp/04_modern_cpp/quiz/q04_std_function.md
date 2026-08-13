# Q4 - `std::function`

What does `std::function<void(double)>` store, and what is the tradeoff?

<details>
<summary>Answer</summary>

It stores any callable that can be invoked with one `double` argument and returns `void`: a function pointer, lambda, functor, or bound member function.

The tradeoff is type-erased dispatch and possible allocation. It is useful at callback boundaries, but templates or concrete callable types are better for hot inner loops.

</details>
