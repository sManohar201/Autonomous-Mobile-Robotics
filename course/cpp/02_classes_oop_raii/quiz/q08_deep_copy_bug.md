# Q8 — Deep Copy Bug

## Find-the-Bug Questions

The following class is supposed to manage a named array of doubles. Find all the bugs and explain how to fix each one.

```cpp
class NamedArray {
public:
    NamedArray(const char* name, std::size_t size)
        : name_{name}, size_{size}, data_{new double[size]}
    {}

    ~NamedArray() { delete[] data_; }

    double& operator[](std::size_t i) { return data_[i]; }

    void print() const {
        std::cout << name_ << ": ";
        for (std::size_t i = 0; i < size_; ++i)
            std::cout << data_[i] << " ";
        std::cout << "\n";
    }

private:
    const char* name_;
    std::size_t size_;
    double*     data_;
};

int main() {
    NamedArray a{"accel", 3};
    a[0] = 1.0; a[1] = 2.0; a[2] = 3.0;

    NamedArray b = a;   // copy
    b[0] = 99.0;

    a.print();   // should print: accel: 1 2 3
    b.print();   // should print: accel: 99 2 3
}
```

<details>
<summary>Answer</summary>

**Bug 1 — Shallow copy: missing copy constructor.**

The compiler-generated copy constructor copies `data_` by pointer value. After `NamedArray b = a`, both `a.data_` and `b.data_` point to the same heap array. `b[0] = 99.0` modifies both arrays. When either object is destroyed, `delete[] data_` runs. When the second is destroyed, it runs again on already-freed memory — **double-free, undefined behaviour**.

**Fix:** Write a copy constructor that allocates a new array and copies the data:

```cpp
NamedArray(const NamedArray& o)
    : name_{o.name_}, size_{o.size_}, data_{new double[o.size_]}
{
    std::copy(o.data_, o.data_ + o.size_, data_);
}
```

**Bug 2 — Missing copy assignment operator.**

Same problem applies to `b = a` (assignment after construction). Must implement copy assignment with a self-assignment guard.

```cpp
NamedArray& operator=(const NamedArray& o) {
    if (this == &o) return *this;
    double* tmp = new double[o.size_];
    std::copy(o.data_, o.data_ + o.size_, tmp);
    delete[] data_;
    data_ = tmp;
    size_ = o.size_;
    name_ = o.name_;
    return *this;
}
```

**Bug 3 — `const char*` name ownership (minor).**

`name_` stores a raw `const char*` pointing into a string literal or caller-owned buffer. If the caller's buffer goes out of scope, `name_` is dangling. Use `std::string` instead:

```cpp
std::string name_;
```

**Summary:** This class violates the Rule of Three. Defining a destructor without also defining copy constructor and copy assignment operator is the bug. As a better long-term fix, replace the raw pointer with `std::vector<double>` and let it handle all memory management automatically.

</details>
