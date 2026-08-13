# 2.6 Move Semantics

**Move semantics** let you transfer ownership of resources instead of copying them — critical for performance when returning large objects or inserting into containers.

### lvalue vs rvalue Intuition

An **lvalue** has a name and a persistent address. An **rvalue** is a temporary with no name — it will cease to exist after the expression. The rule of thumb: *if you can take its address, it's an lvalue.*

```cpp
int x = 5;         // x is lvalue, 5 is rvalue
std::string s = std::string{"hello"};
//           ^^ temporary rvalue
```

### Rvalue References and Move Operations

```cpp
#include <iostream>
#include <cstring>
#include <utility>  // std::move

class PointCloud {
public:
    explicit PointCloud(std::size_t n)
        : size_{n}, points_{new float[n * 3]}
    {
        std::cout << "PointCloud ctor n=" << n << "\n";
    }

    ~PointCloud() {
        std::cout << "PointCloud dtor (points_=" << (void*)points_ << ")\n";
        delete[] points_;
    }

    // Copy constructor — expensive: allocates and copies all points.
    PointCloud(const PointCloud& o)
        : size_{o.size_}, points_{new float[o.size_ * 3]}
    {
        std::copy(o.points_, o.points_ + o.size_ * 3, points_);
        std::cout << "PointCloud COPY ctor\n";
    }

    // Copy assignment.
    PointCloud& operator=(const PointCloud& o) {
        if (this == &o) return *this;
        float* tmp = new float[o.size_ * 3];
        std::copy(o.points_, o.points_ + o.size_ * 3, tmp);
        delete[] points_;
        points_ = tmp;
        size_   = o.size_;
        std::cout << "PointCloud COPY assign\n";
        return *this;
    }

    // Move constructor — cheap: steal the pointer, null out source.
    // Takes an rvalue reference (&&).
    PointCloud(PointCloud&& o) noexcept
        : size_{o.size_}, points_{o.points_}
    {
        o.points_ = nullptr;  // source no longer owns the memory
        o.size_   = 0;
        std::cout << "PointCloud MOVE ctor\n";
    }

    // Move assignment.
    PointCloud& operator=(PointCloud&& o) noexcept {
        if (this == &o) return *this;
        delete[] points_;     // release our current resource
        points_   = o.points_;
        size_     = o.size_;
        o.points_ = nullptr;
        o.size_   = 0;
        std::cout << "PointCloud MOVE assign\n";
        return *this;
    }

    std::size_t size() const { return size_; }

private:
    std::size_t size_;
    float* points_;
};

PointCloud make_cloud(std::size_t n) {
    PointCloud pc{n};
    // ... fill points ...
    return pc;   // NRVO may elide the copy/move entirely
}

int main() {
    PointCloud a{1000};

    // std::move is a cast to rvalue — it doesn't move anything by itself.
    // The move constructor is what actually transfers ownership.
    PointCloud b{std::move(a)};   // move ctor called; a is now empty
    std::cout << "a.size=" << a.size() << " b.size=" << b.size() << "\n";
    // a.size=0  b.size=1000

    PointCloud c{500};
    c = std::move(b);             // move assign; b is now empty

    // Function returning a local object — typically move-elided (NRVO).
    PointCloud d = make_cloud(200);
}
```

### Rule of Five

The Rule of Three extends to the **Rule of Five** when move operations are relevant:

1. Destructor
2. Copy constructor
3. Copy assignment operator
4. Move constructor
5. Move assignment operator

The compiler generates move operations automatically only when you have **not** declared any of {destructor, copy constructor, copy assignment}. As soon as you declare any one of them (as you must for raw-pointer classes), you must explicitly define the move operations too — or `= default` / `= delete` them.

```cpp
class ManagedResource {
public:
    explicit ManagedResource(int id) : id_{id} {}

    ~ManagedResource()                                        = default; // 1
    ManagedResource(const ManagedResource&)                   = default; // 2
    ManagedResource& operator=(const ManagedResource&)        = default; // 3
    ManagedResource(ManagedResource&&) noexcept               = default; // 4
    ManagedResource& operator=(ManagedResource&&) noexcept    = default; // 5

private:
    int id_;
};
```

### Why `std::move` Only Helps Heap-Owning Types

`std::move` is just a cast to an rvalue reference — it doesn't move anything by itself. It tells the compiler "treat this as a temporary you can steal from." Whether that's actually faster depends entirely on what the type owns.

For a `double`, there's nothing to steal — it's 8 bytes sitting directly in the object. Copy and move are identical:

```cpp
double a = 3.14;
double b = std::move(a);   // identical to: double b = a;
// a is still 3.14 — nothing was moved, bytes were just copied
```

For `std::string`, there's a heap allocation to steal:

```cpp
std::string a = "a very long string that lives on the heap...";
//  a:  [ ptr ──→ heap buffer | size | capacity ]

std::string b = std::move(a);
//  b:  [ ptr ──→ heap buffer | size | capacity ]  (same heap block, no copy)
//  a:  [ ptr=null | size=0 | capacity=0 ]          (left empty)
```

Move just transfers the pointer. No heap allocation, no character copy.

| Type | Owns heap memory? | Move faster than copy? |
|---|---|---|
| `double`, `int`, `float` | no | no — same cost |
| `std::string` | yes (usually) | yes |
| `std::vector<T>` | yes | yes |
| `std::unique_ptr` | yes (exclusive) | yes |
| raw pointer `T*` | no (just an address) | no |
| `PointCloud` (above) | yes (`points_` on heap) | yes |

In practice: when constructing a class, `std::move` string and vector members in the constructor, don't bother for numeric types:

```cpp
class Sensor {
public:
    Sensor(std::string name, double rate_hz)
        : name_{std::move(name)}   // moves the heap buffer — one allocation total
        , rate_hz_{rate_hz}        // double: move = copy, no difference
    {}
private:
    std::string name_;
    double rate_hz_;
};
```

### Why `noexcept` on Move Operations

`noexcept` on move operations is not about correctness — it's about enabling an optimisation in the standard library, specifically `std::vector`.

When a `std::vector` runs out of capacity it allocates a bigger buffer and transfers all existing elements. It has a safety rule: if anything goes wrong during the transfer, the original vector must be left unchanged.

- With **copy**: if a copy throws midway, the originals are untouched — easy to roll back.
- With **move**: if a move throws midway, the original elements have already been gutted — nothing to roll back.

So `std::vector` asks: *can this move throw?*

```cpp
// WITHOUT noexcept on PointCloud's move ctor:
//   vector falls back to COPYING every PointCloud during reallocation
//   → expensive heap allocations for every element

// WITH noexcept on PointCloud's move ctor:
//   vector MOVES every PointCloud — just pointer swaps, near zero cost
std::vector<PointCloud> clouds;
clouds.push_back(PointCloud{1000});   // triggers reallocation
```

A well-written move only swaps pointers and integers — it genuinely cannot throw. `noexcept` is your promise to the compiler, and in return the standard library trusts you enough to use moves:

```cpp
PointCloud(PointCloud&& o) noexcept   // promised: no throw
    : size_{o.size_}, points_{o.points_}
{
    o.points_ = nullptr;
    o.size_   = 0;
    // no allocations, no exceptions possible — promise is valid
}
```

If you break the promise and it does throw, `std::terminate()` is called immediately.

### NRVO — Named Return Value Optimisation

When you return a **named local variable** by value, the compiler is allowed to construct it directly in the caller's memory — eliminating the move or copy entirely:

```cpp
PointCloud make_cloud(int n) {
    PointCloud result(n);   // named local variable
    // ... fill it ...
    return result;           // NRVO: compiler constructs result directly where
}                            //       the caller's variable lives — zero copies

PointCloud cloud = make_cloud(1000);   // no move, no copy — one construction total
```

There are two variants:

```cpp
// RVO (unnamed temporary) — guaranteed since C++17, compiler MUST elide
PointCloud make_cloud() {
    return PointCloud(100);   // unnamed — guaranteed zero copies
}

// NRVO (named local) — compiler is allowed but not required to elide
PointCloud make_cloud() {
    PointCloud result(100);   // has a name
    return result;            // usually elided in practice
}
```

NRVO breaks when the compiler can't determine at compile time which object will be returned:

```cpp
PointCloud make_cloud(bool big) {
    PointCloud small_cloud(10);
    PointCloud big_cloud(1000);
    return big ? big_cloud : small_cloud;  // two candidates — can't pre-construct either
}
// Falls back to a move
```

**Practical consequence:** return objects by value freely. You don't need output parameters or heap allocations to avoid copies:

```cpp
// Don't do this "to avoid copies" — NRVO already handles it
void make_cloud(int n, PointCloud* out) { ... }   // unnecessary

// Just do this — idiomatic, clean, equally fast
PointCloud make_cloud(int n) { ... }
```

**Key takeaway:** Mark move constructor and move assignment `noexcept` — `std::vector` and other containers use move only when guaranteed not to throw, so omitting `noexcept` silently forces copying. Return objects by value freely — NRVO eliminates the copy in most cases.
