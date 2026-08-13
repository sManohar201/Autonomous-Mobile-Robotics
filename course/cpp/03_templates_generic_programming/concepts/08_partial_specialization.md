# 3.7 Partial Specialization

Function templates cannot be partially specialized, but class templates can.

```cpp
template <typename T>
struct IsPointer {
    static constexpr bool value = false;
};

template <typename T>
struct IsPointer<T*> {
    static constexpr bool value = true;
};
```

`IsPointer<int>::value` is false. `IsPointer<int*>::value` is true.

Partial specialization is useful when behavior depends on a type shape: pointer, reference, array, optional, vector, or matrix expression.
