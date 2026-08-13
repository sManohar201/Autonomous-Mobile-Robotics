# 3.4 Non-Type Template Parameters

Templates can take values as parameters, not just types.

```cpp
template <typename T, std::size_t N>
class StaticVector {
public:
    T& operator[](std::size_t i) { return data_[i]; }
    const T& operator[](std::size_t i) const { return data_[i]; }
    constexpr std::size_t size() const { return N; }

private:
    T data_[N]{};
};

StaticVector<double, 3> accel;
```

The size is part of the type. This is useful when dimensions are fixed by physics:

- `Vector<double, 3>` for acceleration
- `Matrix<double, 6, 6>` for EKF covariance
- `RingBuffer<ImuSample, 512>` for a fixed-size embedded queue

Compile-time dimensions prevent entire classes of runtime mismatch bugs.
