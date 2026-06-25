// Exercise 03 (Hard) — Pointer Arithmetic, Dangling References, Const Overloads,
//                      and Function Pointers
// ─────────────────────────────────────────────────────────────────────────────

// Q1. Is `arr + N` (one-past-the-end of arr[N]) a valid pointer in C++?
//     Yes. Forming a pointer to one past the end is valid. You CANNOT dereference it.
//     Its only legal use: comparison with other valid pointers in the same array
//     (e.g., as a sentinel in while(ptr != end) loops).

// Q2. Four ways a dangling reference/pointer can arise:
//     1. Return a reference to a local variable (destroyed on function return).
//     2. Store a pointer to a temporary (destroyed at end of full-expression).
//     3. Store a pointer to an element of a std::vector that later reallocates.
//     4. Keep a raw pointer to an object that was deleted (via unique_ptr reset or delete).

// Q3. Function pointer variable declaration for void f(double, int):
//     void (*fp)(double, int);

// Q4. const and non-const overloads:
//     const ImuSample& front() const  — called when the ImuWindow object is const.
//     ImuSample&       front()        — called when the object is non-const.
//     The compiler selects based on the const-ness of *this.

// Q5. After `int a[5]; int* p = a; p += 3;`
//     p is valid (points to a[3]). You can dereference and do p += 1 (→ a[4]).
//     Doing `p += 3` a SECOND time would put p at a[6] — two past the end.
//     Forming this pointer is UB (may only go ONE past the end). Dereferencing is also UB.

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Dangling reference diagnosis
// ─────────────────────────────────────────────────────────────────────────────

// A1: BUG — return reference to local variable s. s is destroyed when the function
//     returns. The returned reference is dangling. Any use is UB.

// A2: NOT A BUG — data_[0] is a member of the ImuWindow object w. As long as w
//     stays alive, the reference is valid. Since w is declared in the same scope
//     as the reference, the lifetime is fine.

// A3: BUG — latest is set to the address of s, which is a local variable.
//     After the closing brace, s is destroyed. latest is now a dangling pointer.
//     latest->ax is UB (may crash, may read garbage).

// A4: BUG — ImuWindow{} is a temporary. back() returns a reference to an internal
//     member of that temporary. The temporary is destroyed at the end of the
//     full-expression (the semicolon). ref is immediately dangling.

// ─────────────────────────────────────────────────────────────────────────────
// PART B — Implement ImuWindow (fixed-capacity ring buffer)
// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <cassert>

struct ImuSample {
    double timestamp_s = 0.0;
    float  ax = 0.f, ay = 0.f, az = 0.f;
    float  gx = 0.f, gy = 0.f, gz = 0.f;
};

class ImuWindow {
public:
    static constexpr int CAPACITY = 8;

    void push_back(const ImuSample& s) {
        data_[head_] = s;
        head_ = (head_ + 1) % CAPACITY;
        if (size_ < CAPACITY) ++size_;
    }

    // Logical index: 0 = oldest, size_-1 = newest.
    // Physical index: (head_ - size_ + i + CAPACITY) % CAPACITY
    const ImuSample& operator[](int i) const {
        return data_[(head_ - size_ + i + CAPACITY) % CAPACITY];
    }

    ImuSample& operator[](int i) {
        // Delegate to const version via const_cast to avoid code duplication.
        return const_cast<ImuSample&>(static_cast<const ImuWindow&>(*this)[i]);
    }

    const ImuSample& front() const { return (*this)[0]; }
    ImuSample&       front()       { return (*this)[0]; }
    const ImuSample& back()  const { return (*this)[size_ - 1]; }
    ImuSample&       back()        { return (*this)[size_ - 1]; }

    int  size()  const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full()  const { return size_ == CAPACITY; }

    void apply_all(void (*fn)(ImuSample&)) {
        for (int i = 0; i < size_; ++i) {
            fn((*this)[i]);
        }
    }

    // Print all CAPACITY physical slots using pointer arithmetic.
    void print_raw(std::ostream& os) const {
        const ImuSample* ptr = data_;
        const ImuSample* end = data_ + CAPACITY;
        int idx = 0;
        while (ptr != end) {
            os << "  [" << idx << "] t=" << std::fixed << std::setprecision(3)
               << ptr->timestamp_s << " ax=" << ptr->ax << "\n";
            ++ptr;
            ++idx;
        }
    }

private:
    ImuSample data_[CAPACITY] = {};
    int size_ = 0;
    int head_ = 0;  // index of NEXT write slot
};

void scale_ax(ImuSample& s) {
    s.ax *= 2.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// EXPECTED OUTPUT:
//   pushed 4, size=4
//   [0] t=0.000 ax=1.000
//   [1] t=0.100 ax=2.000
//   [2] t=0.200 ax=3.000
//   [3] t=0.300 ax=4.000
//   front ax=1.00000, back ax=4.00000
//   push 6 more (wraps capacity=8):
//   size=8, front ax=3.000, back ax=10.000
//   after scale_ax (apply_all *2): front ax=6.00000
//   raw buffer (8 physical slots):
//     [0] t=0.600 ax=14.000
//     [1] t=0.700 ax=16.000
//     [2] t=0.800 ax=18.000
//     [3] t=0.900 ax=20.000
//     [4] t=0.200 ax=6.000
//     [5] t=0.300 ax=8.000
//     [6] t=0.400 ax=10.000
//     [7] t=0.500 ax=12.000
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    ImuWindow win;

    // Push 4 samples
    for (int i = 0; i < 4; ++i) {
        ImuSample s;
        s.timestamp_s = i * 0.1;
        s.ax = static_cast<float>(i + 1);
        win.push_back(s);
    }

    std::cout << "pushed 4, size=" << win.size() << "\n";
    std::cout << std::fixed << std::setprecision(3);
    for (int i = 0; i < win.size(); ++i) {
        std::cout << "  [" << i << "] t=" << win[i].timestamp_s
                  << " ax=" << win[i].ax << "\n";
    }

    std::cout << std::setprecision(5);
    std::cout << "front ax=" << win.front().ax
              << ", back ax=" << win.back().ax << "\n";

    // Push 6 more — wraps around capacity=8
    std::cout << "push 6 more (wraps capacity=8):\n";
    for (int i = 4; i < 10; ++i) {
        ImuSample s;
        s.timestamp_s = i * 0.1;
        s.ax = static_cast<float>(i + 1);
        win.push_back(s);
    }

    std::cout << std::setprecision(3);
    std::cout << "size=" << win.size()
              << ", front ax=" << win.front().ax
              << ", back ax="  << win.back().ax  << "\n";

    // Apply scale_ax (function pointer) to every element
    win.apply_all(scale_ax);
    std::cout << std::setprecision(5);
    std::cout << "after scale_ax (apply_all *2): front ax=" << win.front().ax << "\n";

    // Print all 8 raw physical slots
    std::cout << "raw buffer (8 physical slots):\n";
    win.print_raw(std::cout);

    return 0;
}
