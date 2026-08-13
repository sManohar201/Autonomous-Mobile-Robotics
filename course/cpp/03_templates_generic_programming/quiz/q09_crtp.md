# Q9 - CRTP

What does CRTP provide, and what is its main limitation compared with virtual functions?

<details>
<summary>Answer</summary>

CRTP provides compile-time polymorphism. The base class can call methods on the derived type using `static_cast<Derived&>(*this)`, avoiding virtual dispatch.

Its main limitation is that it does not provide runtime polymorphism. You cannot use CRTP alone to store unrelated derived types in one homogeneous runtime collection through a base pointer.

</details>
