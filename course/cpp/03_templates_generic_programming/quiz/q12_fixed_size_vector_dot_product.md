# Q12 - Fixed-Size Vector Dot Product

Complete the generic dot product.

```cpp
template <typename T, std::size_t N>
class Vector {
public:
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }

private:
    T data_[N]{};
};

template <typename T, std::size_t N>
T dot(const Vector<T, N>& a, const Vector<T, N>& b) {
    /* TODO */
}
```

<details>
<summary>Answer</summary>

```cpp
template <typename T, std::size_t N>
T dot(const Vector<T, N>& a, const Vector<T, N>& b) {
    T sum{};
    for (std::size_t i = 0; i < N; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}
```

The dimension is part of the type, so `dot(Vector<double, 2>, Vector<double, 3>)` does not match this overload.

</details>
