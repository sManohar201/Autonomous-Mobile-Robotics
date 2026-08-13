# Q6 - `std::variant`

When is `std::variant` a better fit than a virtual base class?

<details>
<summary>Answer</summary>

Use `variant` when the set of possible alternatives is fixed and known at compile time. It keeps storage value-based and makes handling cases explicit through `std::visit`.

Use a virtual base class when alternatives need to be extended dynamically, loaded as plugins, or stored behind a stable runtime interface.

</details>
