# **Q6.** What does this print?

## Part B — Predict the Output

```cpp
#include <iostream>
#include <cstdint>
int main() {
    uint8_t x = 200;
    uint8_t y = 100;
    uint8_t z = x + y;
    std::cout << static_cast<int>(z) << "\n";
}
```

<details>
<summary>Answer</summary>

**44** — `200 + 100 = 300`. `uint8_t` holds 0–255. `300 % 256 = 44`. Silent wraparound.

</details>
