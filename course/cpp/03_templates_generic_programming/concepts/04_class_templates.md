# 3.3 Class Templates

A class template describes a family of types.

```cpp
template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(std::size_t capacity)
        : capacity_{capacity}, data_{new T[capacity]}
    {}

    ~RingBuffer() { delete[] data_; }

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

private:
    std::size_t capacity_;
    T* data_;
};
```

`RingBuffer<ImuSample>` and `RingBuffer<double>` are distinct types. A function taking `RingBuffer<double>` cannot accidentally receive `RingBuffer<int>`.

Class templates are the backbone of generic containers, filters, math types, and message adapters.
