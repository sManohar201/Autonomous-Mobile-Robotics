# Q6 — Member Initializer List Order

What does this program print? Explain the surprising result.

```cpp
#include <iostream>

class Range {
public:
    Range(int low, int high)
        : size_{high - low}   // MIL entry 1
        , low_{low}           // MIL entry 2
        , high_{high}         // MIL entry 3
    {}

    void print() const {
        std::cout << "low="  << low_
                  << " high=" << high_
                  << " size=" << size_ << "\n";
    }

private:
    int low_;     // declared first
    int high_;    // declared second
    int size_;    // declared third
};

int main() {
    Range r{2, 10};
    r.print();
}
```

<details>
<summary>Answer</summary>

```
low=2 high=10 size=8
```

Wait — is this actually surprising? Let's look more carefully.

Members are initialized in **declaration order**: `low_`, then `high_`, then `size_`. The MIL lists them in a different order (`size_` first, then `low_`, then `high_`), but the initialization order follows the **declaration order** regardless.

So the actual initialization sequence is:
1. `low_`  = `low` (= 2) ✓
2. `high_` = `high` (= 10) ✓
3. `size_` = `high - low` (= 8) ✓

This works correctly here because `size_` is initialized last (as declared) using the constructor parameters `high` and `low` — not using `high_` or `low_`. If `size_` were initialized using `high_ - low_` (the member variables rather than the parameters), and if `size_` were **declared before** `low_` and `high_`, then `size_` would use the uninitialized members and produce undefined behaviour.

**The lesson:** The MIL order that matters is **declaration order**. Write the MIL in declaration order and use compiler flag `-Wreorder` to catch mismatches. Never initialize a member using another member that is declared after it.

</details>
