# Q6 - Partial Specialization

Can function templates be partially specialized? What should you use instead?

<details>
<summary>Answer</summary>

Function templates cannot be partially specialized. Use overloads, constraints/concepts, or helper class templates that can be partially specialized.

Class templates can be partially specialized:

```cpp
template <typename T>
struct IsPointer { static constexpr bool value = false; };

template <typename T>
struct IsPointer<T*> { static constexpr bool value = true; };
```

</details>
