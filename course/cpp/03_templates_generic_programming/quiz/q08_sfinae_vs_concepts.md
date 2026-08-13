# Q8 - SFINAE vs Concepts

What problem did SFINAE solve, and why are C++20 concepts usually clearer?

<details>
<summary>Answer</summary>

SFINAE lets invalid template substitutions remove an overload from consideration instead of producing a hard compiler error. It enabled constrained templates before C++20.

Concepts express requirements directly and name them:

```cpp
template <std::floating_point T>
T reciprocal(T x);
```

Diagnostics are usually shorter and more directly tied to the failed requirement.

</details>
