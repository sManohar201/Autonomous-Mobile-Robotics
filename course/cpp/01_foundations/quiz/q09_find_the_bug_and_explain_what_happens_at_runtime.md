# **Q9.** Find the bug and explain what happens at runtime.

## Part C — Find the Bug

```cpp
void print_first(const int* arr, size_t size) {
    std::cout << arr[0] << "\n";
}
int main() {
    int* p = nullptr;
    print_first(p, 0);
}
```

<details>
<summary>Answer</summary>

Null pointer dereference. `p` is `nullptr`. `arr[0]` dereferences it unconditionally — undefined behaviour, typically a segfault. Fix: guard with `if (!arr || size == 0) return;`.

</details>
