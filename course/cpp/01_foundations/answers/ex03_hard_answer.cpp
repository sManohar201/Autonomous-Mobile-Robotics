// ANSWER — Exercise 03 (Hard): Pointer Arithmetic, Dangling References,
//                              Const Overloads, and Function Pointers
// ─────────────────────────────────────────────────────────────────────────────

// ── Q1 ────────────────────────────────────────────────────────────────────────
// `arr + N` (one-past-the-end of arr[N]) IS a valid pointer in C++.
// You MAY compute it and compare it to other pointers (e.g., ptr != end).
// You MAY NOT dereference it — that is undefined behavior.
// Its only legal uses are: comparison (ptr < end, ptr == end) and as the
// endpoint of a pointer range in idiomatic loops like:
//   const T* end = arr + N;
//   while (ptr != end) { ... ++ptr; }

// ── Q2 ────────────────────────────────────────────────────────────────────────
// Four ways a dangling reference/pointer can arise:
//   1. Returning a reference to a local variable (destroyed on return).
//   2. Storing a pointer to a local variable; the scope ends but the pointer
//      survives and is later dereferenced.
//   3. Binding a reference to a temporary through a member access:
//        const T& r = SomeClass{}.member();  // SomeClass{} destroyed instantly
//   4. Iterator/pointer invalidation: inserting into a std::vector reallocates
//      its internal buffer, invalidating all existing iterators and pointers
//      into it. A stored T* or T& becomes dangling after the reallocation.

// ── Q3 ────────────────────────────────────────────────────────────────────────
// Function pointer variable declaration for `void f(double, int)`:
//
//   void (*fp)(double, int);
//
// Read right-to-left: fp is a pointer to a function taking (double, int)
// and returning void. To call: `fp(3.14, 42);`

// ── Q4 ────────────────────────────────────────────────────────────────────────
// `const ImuSample& front() const`  — const member function: can be called on
//   const objects (or const references). The implicit `this` is a pointer to
//   const ImuWindow. Returns a const reference — caller cannot modify the sample.
//
// `ImuSample& front()`  — non-const member function: can be called only on
//   non-const (mutable) objects. Returns a mutable reference.
//
// The compiler chooses: if the ImuWindow object is const (or accessed via a
// const reference), the const overload is selected. If the object is mutable,
// the non-const overload is preferred (exact match beats const conversion).
//
// Why both are needed: apply_all, sensor calibration routines, or any code
// that needs to MODIFY elements via front()/back() requires the non-const
// overload. Const correctness ensures read-only contexts cannot accidentally
// mutate data.

// ── Q5 ────────────────────────────────────────────────────────────────────────
// `int a[5]; int* p = a; p += 3;` — p now points to a[3]. Valid, dereferenceable.
//
// `p += 3` again → p points to a[6]. INVALID. The array has only elements
// [0..4], so the only valid "extra" address is a+5 (one-past-end). Computing
// a+6 is UNDEFINED BEHAVIOR — even forming the address is UB, not just
// dereferencing it. The compiler may optimize assuming you never do this.
//
// Rule: pointer arithmetic is defined only within an array (or one-past-end).
// Going beyond one-past-end is UB at the point of the arithmetic, not just
// at the dereference.

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Dangling reference / pointer diagnosis
// ─────────────────────────────────────────────────────────────────────────────

// A1:
//   const ImuSample& get_gravity_sample() {
//       ImuSample s{0.0, 9.81f};
//       return s;
//   }
//
// BUG: `s` is a local variable with automatic storage duration. It is
// destroyed when the function returns. The returned reference is immediately
// dangling. Any use of the reference by the caller is undefined behavior.
// GCC/Clang warn: "reference to local variable 's' returned".

// A2:
//   class ImuWindow { ImuSample data_[8]; int size_; public:
//       const float& first_ax() const { return data_[0].ax; }
//   };
//   const ImuWindow w; const float& ax = w.first_ax();
//
// SAFE. first_ax() returns a reference to data_[0].ax, which is a MEMBER of
// the ImuWindow object `w`. As long as `w` is alive (it is here — it lives
// in the enclosing scope), the reference is valid. The ImuWindow object is not
// a temporary; it is a named local variable.

// A3:
//   ImuSample* latest = nullptr;
//   { ImuSample s{1.0, 2.0f}; latest = &s; }
//   std::cout << latest->ax;
//
// BUG: `s` has automatic storage duration limited to the inner block `{}`.
// When the `}` is reached, `s` is destroyed. `latest` still holds the old
// address, which is now a dangling pointer. Dereferencing `latest->ax` after
// the block is undefined behavior. On the stack, the memory may appear to
// contain the old value (common in debug builds), or may be overwritten by a
// subsequent call frame (common in release builds). This bug is real and
// common in sensor callback code that stores pointers to message objects.

// A4:
//   const ImuSample& ref = ImuWindow{}.back();
//
// BUG: `ImuWindow{}` creates a temporary. `back()` returns a reference to
// `data_[size_-1]` inside that temporary. The temporary's lifetime is NOT
// extended here because the reference is bound to a MEMBER of the temporary,
// not to the temporary object itself. Lifetime extension (C++17 §15.2) only
// applies when a const reference is bound directly to the temporary object;
// it does NOT apply through member access. The temporary ImuWindow is
// destroyed at the semicolon, and `ref` immediately becomes a dangling
// reference. GCC/Clang may warn: "binding reference to temporary".
//
// Contrast with: `const ImuSample& ref = ImuSample{1.0, 9.81f};` — here
// the temporary IS the bound object, so its lifetime IS extended. That is safe.

// ─────────────────────────────────────────────────────────────────────────────

#include <iostream>
#include <iomanip>
#include <algorithm>
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
        // When size_ == CAPACITY (already full), head_ advances and overwrites
        // the oldest slot. size_ stays at CAPACITY.
    }

    // Physical index for logical index i (0 = oldest):
    //   phys = (head_ - size_ + i + CAPACITY) % CAPACITY
    //
    // Derivation: head_ - size_ gives the physical index of the OLDEST element
    // (the slot just AFTER the one most recently overwritten). Adding i gives
    // the i-th logical element. The + CAPACITY ensures we stay non-negative
    // before the modulo.
    const ImuSample& operator[](int i) const {
        int phys = (head_ - size_ + i + CAPACITY) % CAPACITY;
        return data_[phys];
    }

    // Non-const overload: needed so callers on mutable ImuWindow can modify
    // elements (e.g., apply calibration offsets in place).
    // Delegates to the const version via const_cast — the canonical pattern
    // to avoid duplicating the index arithmetic.
    ImuSample& operator[](int i) {
        return const_cast<ImuSample&>(
            static_cast<const ImuWindow&>(*this)[i]
        );
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

    // Print every physical slot using pointer arithmetic.
    // data_ + CAPACITY is the one-past-end address — valid to form and compare,
    // but NOT to dereference. The while loop never dereferences `end`.
    void print_raw(std::ostream& os) const {
        const ImuSample* ptr = data_;
        const ImuSample* end = data_ + CAPACITY;
        int slot = 0;
        while (ptr != end) {
            os << std::fixed << std::setprecision(3);
            os << "  [" << slot << "] t=" << ptr->timestamp_s
               << " ax=" << ptr->ax << "\n";
            ++ptr;
            ++slot;
        }
    }

private:
    ImuSample data_[CAPACITY] = {};
    int size_ = 0;
    int head_ = 0;
};

// Function pointer demo — passed to apply_all.
void scale_ax(ImuSample& s) {
    s.ax *= 2.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// EXPECTED OUTPUT:
//   pushed 4, size=4
//     [0] t=0.000 ax=1.000
//     [1] t=0.100 ax=2.000
//     [2] t=0.200 ax=3.000
//     [3] t=0.300 ax=4.000
//   front ax=1.00000, back ax=4.00000
//   push 6 more (wraps capacity=8):
//   size=8, front ax=3.000, back ax=10.000
//   after scale_ax (apply_all *2): front ax=6.00000
//   raw buffer (8 physical slots):
//     [0] t=0.800 ax=18.000
//     [1] t=0.900 ax=20.000
//     [2] t=0.200 ax=6.000
//     [3] t=0.300 ax=8.000
//     [4] t=0.400 ax=10.000
//     [5] t=0.500 ax=12.000
//     [6] t=0.600 ax=14.000
//     [7] t=0.700 ax=16.000
// ─────────────────────────────────────────────────────────────────────────────

int main() {
    ImuWindow win;

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

    win.apply_all(scale_ax);
    std::cout << std::setprecision(5);
    std::cout << "after scale_ax (apply_all *2): front ax=" << win.front().ax << "\n";

    std::cout << "raw buffer (8 physical slots):\n";
    win.print_raw(std::cout);

    return 0;
}
