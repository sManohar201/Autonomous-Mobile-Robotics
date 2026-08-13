# Q1 — Rule of Three / Rule of Five

## Conceptual Questions

A class `SensorBuffer` owns a raw `float*` array allocated on the heap. You write a custom destructor to `delete[]` it.

(a) Why does the compiler-generated copy constructor cause undefined behaviour for this class?
(b) List the five special member functions that form the Rule of Five.
(c) If you don't need copying at all, what is the cleanest way to express that?

<details>
<summary>Answer</summary>

**(a)** The compiler-generated copy constructor performs a **shallow copy**: it copies the pointer value, not the data it points to. After `SensorBuffer b = a;`, both `a.data_` and `b.data_` point to the same heap allocation. When `b` is destroyed it calls `delete[] b.data_`. When `a` is then destroyed it calls `delete[] a.data_` on the same memory — this is a **double-free**, which is undefined behaviour and typically a crash.

**(b)** The Rule of Five:
1. Destructor
2. Copy constructor
3. Copy assignment operator
4. Move constructor
5. Move assignment operator

**(c)** Delete the copy constructor and copy assignment operator explicitly:

```cpp
SensorBuffer(const SensorBuffer&)            = delete;
SensorBuffer& operator=(const SensorBuffer&) = delete;
```

This makes any accidental copy a **compile error** instead of a runtime crash. You would also typically `= default` or implement the move operations so the object is still transferable.

</details>
