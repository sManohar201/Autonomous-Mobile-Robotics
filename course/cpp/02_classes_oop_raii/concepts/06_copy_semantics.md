# 2.5 Copy Semantics

When you copy an object, C++ calls the **copy constructor** (initialization from another object) or **copy assignment operator** (assignment after both objects exist).

### Shallow Copy and the Double-Free Bug

If your class owns heap memory through a raw pointer, the compiler-generated copy constructor copies the pointer value — both objects then point to the same allocation. Destroying either one leaves the other with a dangling pointer; destroying both is undefined behaviour (double-free).

```cpp
#include <cstring>
#include <iostream>
#include <stdexcept>

// Dangerous: raw pointer without custom copy semantics.
class BadBuffer {
public:
    explicit BadBuffer(std::size_t size)
        : size_{size}, data_{new double[size]}
    {
        std::fill(data_, data_ + size_, 0.0);
    }

    ~BadBuffer() {
        delete[] data_;   // problem: if data_ is shared, this is a double-free
    }

    // No copy constructor or copy assignment defined.
    // The compiler generates:
    //   BadBuffer(const BadBuffer& o) : size_{o.size_}, data_{o.data_} {}
    // This is a SHALLOW copy: both objects share the same data_ pointer.

    double* data() { return data_; }

private:
    std::size_t size_;
    double* data_;
};

// void demonstrate_bug() {
//     BadBuffer a{4};
//     BadBuffer b{a};   // shallow copy: b.data_ == a.data_
//     // When b is destroyed: delete[] b.data_  -> fine
//     // When a is destroyed: delete[] a.data_  -> DOUBLE FREE: undefined behaviour
// }
```

### Rule of Three: Deep Copy Done Right

If you define any of {**destructor**, **copy constructor**, **copy assignment operator**}, you almost certainly need all three.

```cpp
class Buffer {
public:
    explicit Buffer(std::size_t size)
        : size_{size}, data_{new double[size]}
    {
        std::fill(data_, data_ + size_, 0.0);
        std::cout << "Buffer ctor, size=" << size_ << "\n";
    }

    // Destructor (1 of 3)
    ~Buffer() {
        std::cout << "Buffer dtor, size=" << size_ << "\n";
        delete[] data_;
    }

    // Copy constructor (2 of 3) — deep copy
    Buffer(const Buffer& other)
        : size_{other.size_}, data_{new double[other.size_]}
    {
        std::copy(other.data_, other.data_ + other.size_, data_);
        std::cout << "Buffer copy ctor\n";
    }

    // Copy assignment operator (3 of 3) — deep copy
    Buffer& operator=(const Buffer& other) {
        if (this == &other) return *this;   // self-assignment guard

        // Allocate new memory before releasing old (strong exception safety).
        double* tmp = new double[other.size_];
        std::copy(other.data_, other.data_ + other.size_, tmp);

        delete[] data_;      // release old resource
        data_ = tmp;
        size_ = other.size_;

        std::cout << "Buffer copy assign\n";
        return *this;
    }

    std::size_t size() const { return size_; }
    double& operator[](std::size_t i) { return data_[i]; }
    double  operator[](std::size_t i) const { return data_[i]; }

private:
    std::size_t size_;
    double* data_;
};

int main() {
    Buffer a{4};
    a[0] = 1.0; a[1] = 2.0;

    // ── Copy constructor ─────────────────────────────────────────────────────
    // Called when a NEW object is being initialized from an existing one.
    Buffer b{a};       // same as: Buffer b = a;
    b[0] = 99.0;

    std::cout << "a[0]=" << a[0] << "  b[0]=" << b[0] << "\n";
    // a[0]=1   b[0]=99  — independent allocations, no aliasing

    // ── Copy assignment operator ─────────────────────────────────────────────
    // Called when BOTH objects already exist and you assign one to the other.
    Buffer c{4};       // c already exists — has its own allocation
    c[0] = 7.0;

    c = a;             // copy assignment: c releases its old memory,
                       // allocates new memory, copies a's data into it
    c[0] = 55.0;

    std::cout << "a[0]=" << a[0] << "  c[0]=" << c[0] << "\n";
    // a[0]=1   c[0]=55  — still independent

    // ── Self-assignment (must be safe) ───────────────────────────────────────
    a = a;             // the self-assignment guard handles this — no crash
    std::cout << "a[0] after self-assign=" << a[0] << "\n";
    // a[0]=1  — unchanged
}
```

### Self-Assignment Guard — When Is It Actually Required?

You rarely write `a = a` deliberately. But it happens **indirectly**:

```cpp
buffers[i] = buffers[j];         // what if i == j at runtime?

void update(Buffer& dst, const Buffer& src) { dst = src; }
update(buffers[idx], buffers[idx]);   // same object for both arguments
```

Whether the guard is **required** depends on which pattern you use:

**Pattern 1 — delete first, then allocate (guard required):**
```cpp
Buffer& operator=(const Buffer& other) {
    delete[] data_;                                   // 1. free our memory
    size_ = other.size_;
    data_ = new double[other.size_];                  // 2. allocate
    std::copy(other.data_, ..., data_);               // 3. copy from other.data_
    return *this;
    // In self-assignment: step 1 frees data_ which IS other.data_.
    // Step 3 copies from freed memory — undefined behaviour.
    // The guard is REQUIRED here.
}
```

**Pattern 2 — allocate tmp first, then delete (guard not required, but kept as optimisation):**
```cpp
Buffer& operator=(const Buffer& other) {
    double* tmp = new double[other.size_];            // 1. allocate new block
    std::copy(other.data_, ..., tmp);                 // 2. copy into tmp — other.data_ still alive
    delete[] data_;                                   // 3. free old block
    data_ = tmp;
    size_ = other.size_;
    return *this;
    // In self-assignment: tmp is filled from other.data_ BEFORE we delete anything.
    // Correctness is preserved even without the guard.
    // Guard is kept only as a cheap short-circuit (skip unnecessary work).
}
```

The `Buffer` example above uses Pattern 2, so the guard is a performance optimisation, not a correctness requirement. Pattern 1 — which you'll see in simpler implementations — needs it for safety.

**Key takeaway:** Any class owning a raw pointer must implement the Rule of Three. In practice, prefer `std::vector` or a smart pointer, which implement these correctly for you.
