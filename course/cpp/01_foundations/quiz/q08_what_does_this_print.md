# **Q8.** What does this print?

```cpp
#include <iostream>
namespace A { int x = 1; }
namespace B { int x = 2; }
int main() {
    using namespace A;
    std::cout << x << "\n";
    {
        using namespace B;
        std::cout << x << "\n";
    }
    std::cout << x << "\n";
}
```

<details>
<summary>Answer</summary>

Prints `1`, then a **compile error** on the second `cout`. Inside the inner block both `A::x` and `B::x` are in scope — the compiler cannot choose between them (ambiguous). The third line would print `1` if the program compiled.

</details>
