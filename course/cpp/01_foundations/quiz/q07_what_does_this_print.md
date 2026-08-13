# **Q7.** What does this print?

```cpp
#include <iostream>
void increment(int x) { x += 1; }
int main() {
    int a = 5;
    increment(a);
    std::cout << a << "\n";
}
```

<details>
<summary>Answer</summary>

**5** — `increment` receives a copy of `a`. The original is not modified. To modify the caller's variable, take `int&` or `int*`.

</details>
