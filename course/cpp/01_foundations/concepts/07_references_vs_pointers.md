# 1.6 References vs Pointers

```cpp
int x = 42;
int* ptr = &x;    // pointer: stores the address
int& ref = x;     // reference: another name for x

*ptr = 10;        // modify through pointer
ref  = 10;        // modify through reference — no * needed
```

| Property | Reference | Pointer |
|----------|-----------|---------|
| Can be null | No | Yes (`nullptr`) |
| Can be reseated | No | Yes |
| Must be initialised | Yes | No |
| Arithmetic | No | Yes |

**Use a reference when:** the thing is guaranteed to exist (function parameters, range-for).
**Use a pointer when:** null is a valid state, or you need to swap what you're pointing to.

Never dereference a pointer without checking it:
```cpp
void process(const Scan* scan) {
    if (!scan) return;
    // safe to use scan->...
}
```
