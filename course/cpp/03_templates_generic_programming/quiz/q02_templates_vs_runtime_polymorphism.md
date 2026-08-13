# Q2 - Templates vs Runtime Polymorphism

When would you prefer a template-based sensor utility over a virtual base class? When would a virtual base class be the better choice?

<details>
<summary>Answer</summary>

Prefer templates when the type is known at compile time and you want zero-overhead, type-specific code: fixed-size math, typed buffers, compile-time traits, or algorithms over message structs.

Prefer runtime polymorphism when you need to store different concrete types behind one interface at runtime, such as:

```cpp
std::vector<std::unique_ptr<SensorPlugin>>
```

Templates give compile-time polymorphism. Virtual functions give runtime polymorphism. They solve different problems.

</details>
