// Exercise 03 (Hard) — Pointer Arithmetic, Dangling References, Const Overloads,
//                      and Function Pointers
// ─────────────────────────────────────────────────────────────────────────────
//
// CONTEXT:
//   Real-time robotics requires fixed-capacity, allocation-free data structures.
//   nav2's costmap, robot_localization's filter buffer, and ROS2 sensor drivers
//   all use fixed ring buffers to avoid heap allocation in hot paths. Writing
//   these correctly requires a precise understanding of pointer arithmetic and
//   const-correct interfaces.
//
// ─────────────────────────────────────────────────────────────────────────────
// PRE-CODING RESEARCH QUESTIONS — Answer in the spaces provided.
// ─────────────────────────────────────────────────────────────────────────────

// Q1. Is `arr + N` (one-past-the-end of arr[N]) a valid pointer in C++?
//     Can you dereference it? What is its only legal use?
//
//     YOUR ANSWER:
//     TODO

// Q2. List 4 distinct ways a dangling reference or dangling pointer can arise.
//
//     YOUR ANSWER:
//     TODO

// Q3. Write the declaration of a function pointer variable `fp` that can point
//     to any function with signature `void f(double, int)`.
//
//     YOUR ANSWER (write the declaration here as a C++ line):
//     TODO

// Q4. What is the difference between:
//       const ImuSample& front() const
//       ImuSample&       front()
//     on the same class? When does the compiler call each overload?
//
//     YOUR ANSWER:
//     TODO

// Q5. Given `int a[5] = {1,2,3,4,5}; int* p = a; p += 3;`
//     Is `p` a valid pointer? Can you then do `p += 3` a second time and
//     dereference? Explain why or why not using the C++ standard rules.
//
//     YOUR ANSWER:
//     TODO

// ─────────────────────────────────────────────────────────────────────────────
// PART A — Dangling reference / pointer diagnosis
//
// For each of the 4 cases below, identify:
//   (1) Is there a bug?
//   (2) If so, what exactly is the bug and why is it UB?
//   (3) If not, explain why it is safe.
//
// Answer in comments — NO code changes needed.
// ─────────────────────────────────────────────────────────────────────────────

// A1:
//   struct ImuSample { double t; float ax; };
//   const ImuSample& get_gravity_sample() {
//       ImuSample s{0.0, 9.81f};
//       return s;
//   }
//   // Called as: const ImuSample& ref = get_gravity_sample();
//
// YOUR DIAGNOSIS:
// TODO

// A2:
//   class ImuWindow {
//       ImuSample data_[8];
//       int size_;
//   public:
//       const float& first_ax() const { return data_[0].ax; }
//   };
//   // Called as: const ImuWindow w; const float& ax = w.first_ax();
//
// YOUR DIAGNOSIS:
// TODO

// A3:
//   ImuSample* latest = nullptr;
//   {
//       ImuSample s{1.0, 2.0f};
//       latest = &s;
//   }
//   std::cout << latest->ax;  // usage after scope
//
// YOUR DIAGNOSIS:
// TODO

// A4:
//   struct ImuWindow { ImuSample data_[8]; int size_; ImuSample& back(); };
//   const ImuSample& ref = ImuWindow{}.back();
//   // (ImuWindow{} is a temporary; back() returns a reference to an internal member)
//
// YOUR DIAGNOSIS:
// TODO

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

    // Push one sample. When full, overwrite the oldest entry (circular).
    // Physical write slot: data_[head_]
    // After write: head_ = (head_ + 1) % CAPACITY
    //              size_ = std::min(size_ + 1, CAPACITY)
    //
    // THINK: why is head_ advanced BEFORE capping size_? Draw the state
    // after 9 pushes into a capacity-8 buffer on paper before coding.
    void push_back(const ImuSample& s);

    // Element access. Logical index 0 = oldest, size_-1 = newest.
    // Physical index formula: (head_ - size_ + i + CAPACITY) % CAPACITY
    //
    // THINK: Work through this formula for head_=3, size_=3, i=0 on paper.
    //
    // THINK: Why do BOTH const and non-const versions need to exist?
    //        When would a caller need the mutable (non-const) version?
    const ImuSample& operator[](int i) const;
    ImuSample&       operator[](int i);

    // THINK: What happens if you call front() or back() on an empty window?
    //        Is it UB? Should you add a guard? Consider both safety-critical
    //        (defensive check) and high-performance (precondition, no check) approaches.
    const ImuSample& front() const;  // oldest entry
    ImuSample&       front();
    const ImuSample& back()  const;  // newest entry
    ImuSample&       back();

    int  size()  const { return size_; }
    bool empty() const { return size_ == 0; }
    bool full()  const { return size_ == CAPACITY; }

    // Apply fn to every element in LOGICAL order (oldest to newest, 0..size_-1).
    // fn signature: void fn(ImuSample&)
    //
    // THINK: This takes a plain function pointer, NOT a lambda or std::function.
    //        Write an example regular function (see scale_ax below) and pass it.
    void apply_all(void (*fn)(ImuSample&));

    // Print all CAPACITY physical slots in raw array order.
    // Must use pointer arithmetic (NOT the subscript operator).
    // Pattern: const ImuSample* ptr = data_;
    //          const ImuSample* end = data_ + CAPACITY;
    //          while (ptr != end) { /* print */ ++ptr; }
    //
    // THINK: Is data_ + CAPACITY a valid pointer? Can you dereference it?
    void print_raw(std::ostream& os) const;

private:
    ImuSample data_[CAPACITY] = {};
    int size_ = 0;
    int head_ = 0;  // index of NEXT write slot
};

// ── Function passed to apply_all ─────────────────────────────────────────────
// This doubles the ax field of a sample. Passed as a plain function pointer.
void scale_ax(ImuSample& s) {
    s.ax *= 2.0f;
}

// ── ImuWindow method implementations ─────────────────────────────────────────
// TODO: Replace each stub body with a correct implementation.

void ImuWindow::push_back(const ImuSample& s) {
    (void)s; // TODO: write to data_[head_], advance head_, update size_
}

const ImuSample& ImuWindow::operator[](int i) const {
    // TODO: compute physical index via (head_ - size_ + i + CAPACITY) % CAPACITY
    (void)i;
    return data_[0]; // stub — always returns slot 0
}

ImuSample& ImuWindow::operator[](int i) {
    // TODO: delegate to const version via const_cast, or repeat index formula
    (void)i;
    return data_[0]; // stub
}

const ImuSample& ImuWindow::front() const { return (*this)[0]; }
ImuSample&       ImuWindow::front()       { return (*this)[0]; }
const ImuSample& ImuWindow::back()  const { return (*this)[size_ - 1]; }
ImuSample&       ImuWindow::back()        { return (*this)[size_ - 1]; }

void ImuWindow::apply_all(void (*fn)(ImuSample&)) {
    (void)fn; // TODO: iterate logical indices 0..size_-1, call fn on each
}

void ImuWindow::print_raw(std::ostream& os) const {
    // TODO: use pointer arithmetic — const ImuSample* ptr = data_;
    //       const ImuSample* end = data_ + CAPACITY; while(ptr != end){...++ptr;}
    (void)os; // stub
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
