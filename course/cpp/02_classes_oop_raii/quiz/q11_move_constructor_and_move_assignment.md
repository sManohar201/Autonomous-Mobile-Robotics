# Q11 — Move Constructor and Move Assignment

## Code-Completion Questions

Complete the two missing special member functions for this class. The class owns a heap-allocated array. Your implementations must:
- Transfer ownership without allocating new memory
- Leave the source object in a valid, destructible state
- Be marked `noexcept`

```cpp
class FloatBuffer {
public:
    explicit FloatBuffer(std::size_t n)
        : size_{n}, data_{new float[n]}
    {}

    ~FloatBuffer() { delete[] data_; }

    FloatBuffer(const FloatBuffer& o)
        : size_{o.size_}, data_{new float[o.size_]}
    {
        std::copy(o.data_, o.data_ + o.size_, data_);
    }

    FloatBuffer& operator=(const FloatBuffer& o) {
        if (this == &o) return *this;
        float* tmp = new float[o.size_];
        std::copy(o.data_, o.data_ + o.size_, tmp);
        delete[] data_;
        data_ = tmp;
        size_ = o.size_;
        return *this;
    }

    // TODO: move constructor
    FloatBuffer(FloatBuffer&& o) noexcept
        /* your implementation here */

    // TODO: move assignment operator
    FloatBuffer& operator=(FloatBuffer&& o) noexcept
        /* your implementation here */

    std::size_t size() const { return size_; }
    float* data()            { return data_; }

private:
    std::size_t size_;
    float*      data_;
};
```

<details>
<summary>Answer</summary>

```cpp
// Move constructor — steal the source's resources, null it out.
FloatBuffer(FloatBuffer&& o) noexcept
    : size_{o.size_}, data_{o.data_}
{
    o.data_ = nullptr;   // source no longer owns the memory
    o.size_ = 0;         // leave source in a valid, empty state
}

// Move assignment operator.
FloatBuffer& operator=(FloatBuffer&& o) noexcept {
    if (this == &o) return *this;   // self-move guard (rare but valid)

    delete[] data_;       // release our current resource

    data_   = o.data_;    // steal source's pointer
    size_   = o.size_;

    o.data_ = nullptr;    // null out source
    o.size_ = 0;

    return *this;
}
```

**Key points:**
- Setting `o.data_ = nullptr` is essential. When `o` is destroyed, its destructor calls `delete[] nullptr`, which is a no-op — safe.
- `noexcept` is critical: `std::vector` uses move only when it is `noexcept`. Without it, reallocations fall back to copying.
- The self-move guard (`this == &o`) is technically required by the standard (moved-from state must be valid), though self-move is uncommon in practice.

</details>
