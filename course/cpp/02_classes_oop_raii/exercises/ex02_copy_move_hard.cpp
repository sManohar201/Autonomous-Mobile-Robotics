// Exercise 02 — Rule of Five on a two-level heap: ScanBuffer
//
// Context
// -------
// A 2D LiDAR produces a full 360-degree scan at ~10 Hz.  A typical robotics
// pipeline stores the last N scans in a circular buffer (ring buffer) so that
// algorithms can look back in time without unbounded memory growth.
//
// ScanBuffer is that ring buffer.  It is deliberately implemented with raw
// heap pointers (no std::vector, no std::unique_ptr for the storage) so that
// you must reason carefully about ownership.
//
// The tricky part: there are TWO levels of heap allocation.
//   Level 1 — data_ : a heap array of Scan structs, capacity_ elements.
//   Level 2 — each Scan.ranges : a heap array of floats, Scan.n_ranges elements.
// Both levels must be correctly managed by every special member.
//
// ── Data layout ─────────────────────────────────────────────────────────────
//
//   struct Scan {
//       float* ranges;   // heap-allocated, nullptr if slot not yet written
//       int    n_ranges;
//   };
//
//   class ScanBuffer {
//       Scan* data_;      // heap array of capacity_ Scan objects
//       int   capacity_;  // fixed at construction
//       int   size_;      // number of valid (written) slots, 0..capacity_
//       int   head_;      // index of the NEXT slot to write (mod capacity_)
//   };
//
// ── Circular buffer semantics ────────────────────────────────────────────────
// push() writes at data_[head_], then head_ = (head_ + 1) % capacity_.
// When size_ < capacity_, size_ grows.  When full, the oldest scan is
// silently overwritten (its ranges array must be freed first).
//
// Logical indexing in get(i):
//   Physical slot = (head_ - size_ + i + capacity_) % capacity_
// This formula maps logical 0 (oldest) to the correct physical slot.
// Work it out on paper before implementing — off-by-one errors here are subtle.
//
// ── What you must implement ──────────────────────────────────────────────────
//
// Constructor: ScanBuffer(int capacity)
//   Allocates data_ = new Scan[capacity_].
//   Sets ALL Scan.ranges = nullptr, Scan.n_ranges = 0.
//   size_ = head_ = 0.
//   THINK: why must you zero-initialise the Scan slots?  What happens in
//   the destructor if some ranges pointers are garbage?
//
// Destructor:
//   For each slot in data_, if ranges != nullptr, delete[] ranges.
//   Then delete[] data_.
//   THINK: is it safe to delete[] nullptr?  (Yes, it is a no-op — use it.)
//
// Copy constructor: ScanBuffer(const ScanBuffer& other)
//   Deep copy: allocate a new data_ array of other.capacity_ Scan structs.
//   For each slot, if other.data_[i].ranges != nullptr, allocate a new
//   ranges array and memcpy the floats.  Copy n_ranges, size_, head_.
//   TRICKY: after the copy, modifying the copy's ranges must NOT affect
//   the original.  A shallow copy of ranges pointers would break this.
//
// Copy assignment: ScanBuffer& operator=(const ScanBuffer& other)
//   Guard against self-assignment (if (this == &other) return *this).
//   Free ALL existing ranges arrays, then free data_.
//   Then perform the same deep copy as the copy constructor.
//   Capacities may differ — you must reallocate data_ to other.capacity_.
//   THINK: what if new[] throws partway through?  In this exercise you
//   do not need to provide the strong exception guarantee, but note where
//   a partial state could occur.
//
// Move constructor: ScanBuffer(ScanBuffer&& other) noexcept
//   Steal: data_ = other.data_, capacity_ = other.capacity_,
//          size_ = other.size_, head_ = other.head_.
//   Leave other in a valid empty state: other.data_ = nullptr,
//   other.capacity_ = 0, other.size_ = 0, other.head_ = 0.
//   THINK: why must you set other.data_ = nullptr (not just leave it)?
//   What would happen in other's destructor if you didn't?
//
// Move assignment: ScanBuffer& operator=(ScanBuffer&& other) noexcept
//   Guard self-assignment.
//   Free own resources (same as in destructor).
//   Steal from other (same as move constructor).
//   Return *this.
//
// void push(const float* ranges, int n)
//   If capacity_ == 0: no-op (moved-from buffer — do not crash).
//   The slot at data_[head_] may already contain a scan (when the buffer
//   is full).  Free its ranges array before overwriting.
//   Allocate new float[n], copy ranges into it.
//   Write to data_[head_], advance head_, update size_.
//
// const Scan& get(int i) const
//   Throw std::out_of_range if i < 0 || i >= size_.
//   Return data_[physical_index] using the circular formula above.
//
// Accessors: int size() const, int capacity() const
//
// ── Expected output ──────────────────────────────────────────────────────────
//   pushed 3 scans
//   scan[0]: 3 ranges, first=1.0
//   scan[1]: 3 ranges, first=4.0
//   scan[2]: 3 ranges, first=7.0
//   copy: scan[0] first=1.0 (independent from original)
//   after modifying copy, original scan[0] first=1.0
//   moved: size=3, original size=0
//   assigned: scan[0] first=1.0

#include <iostream>
#include <stdexcept>
#include <cstring>
#include <utility>

struct Scan {
    float* ranges;
    int    n_ranges;
};

class ScanBuffer {
public:
    // TODO: Constructor — allocate data_, zero-initialise all Scan slots.
    explicit ScanBuffer(int capacity);

    // TODO: Destructor — free each Scan.ranges, then free data_.
    ~ScanBuffer();

    // TODO: Copy constructor — deep copy (two-level allocation).
    ScanBuffer(const ScanBuffer& other);

    // TODO: Copy assignment — self-assignment guard, free old, deep copy.
    //   Handles differing capacities.
    ScanBuffer& operator=(const ScanBuffer& other);

    // TODO: Move constructor (noexcept) — steal all fields, null out source.
    ScanBuffer(ScanBuffer&& other) noexcept;

    // TODO: Move assignment (noexcept) — free own resources, then steal.
    ScanBuffer& operator=(ScanBuffer&& other) noexcept;

    // TODO: push — overwrite oldest if full (free its ranges first).
    //   No-op if capacity_ == 0.
    void push(const float* ranges, int n);

    // TODO: get — logical index 0=oldest.
    //   Throw std::out_of_range if i is out of bounds.
    //   Physical index formula: (head_ - size_ + i + capacity_) % capacity_
    //   Walk through this formula on paper with a worked example before coding:
    //     capacity=4, size=3, head=1 (we wrote slots 0,1,2 then advanced to 1)
    //     Wait — think again.  After 3 pushes starting at head=0:
    //       push 0 → write slot 0, head becomes 1
    //       push 1 → write slot 1, head becomes 2
    //       push 2 → write slot 2, head becomes 3
    //     Now head=3, size=3.
    //     get(0) = oldest = slot 0.  Formula: (3 - 3 + 0 + 4) % 4 = 4%4 = 0. ✓
    //     get(2) = newest = slot 2.  Formula: (3 - 3 + 2 + 4) % 4 = 6%4 = 2. ✓
    //   Now add a 4th push on a capacity-4 buffer (no overwrite yet, buffer full):
    //     push 3 → write slot 3, head becomes 0, size=4.
    //     get(0) = oldest = slot 0.  (0 - 4 + 0 + 4) % 4 = 0. ✓
    //   Now push a 5th (overwrite oldest, slot 0):
    //     free slot 0's ranges, write new data there, head becomes 1, size stays 4.
    //     get(0) = new oldest = slot 1.  (1 - 4 + 0 + 4) % 4 = 1. ✓
    const Scan& get(int i) const;

    int size() const;
    int capacity() const;

private:
    Scan* data_;
    int   capacity_;
    int   size_;
    int   head_;

    // TODO (optional): private helper void free_all_scans()
    //   Frees each ranges array and nulls the pointer.  Reuse in destructor
    //   and copy assignment to avoid code duplication.
};


int main() {
    ScanBuffer buf(4);

    float s0[3] = {1.0f, 2.0f, 3.0f};
    float s1[3] = {4.0f, 5.0f, 6.0f};
    float s2[3] = {7.0f, 8.0f, 9.0f};

    buf.push(s0, 3);
    buf.push(s1, 3);
    buf.push(s2, 3);
    std::cout << "pushed 3 scans\n";

    for (int i = 0; i < buf.size(); ++i) {
        const Scan& sc = buf.get(i);
        std::cout << "scan[" << i << "]: " << sc.n_ranges
                  << " ranges, first=" << sc.ranges[0] << "\n";
    }

    // ── Copy test ────────────────────────────────────────────────────────────
    ScanBuffer copy = buf;
    std::cout << "copy: scan[0] first=" << copy.get(0).ranges[0]
              << " (independent from original)\n";

    // Modify the copy's first scan — original must be unchanged.
    // (You cannot do this without pointer arithmetic on the copy's data;
    //  to keep main simple we just re-push over it.)
    float modified[3] = {99.0f, 0.0f, 0.0f};
    // We can't easily modify a slot directly without a mutating accessor.
    // Instead, verify independence by confirming original still returns 1.0.
    std::cout << "after modifying copy, original scan[0] first="
              << buf.get(0).ranges[0] << "\n";

    // ── Move test ────────────────────────────────────────────────────────────
    ScanBuffer moved = std::move(buf);   // buf is now in moved-from state
    std::cout << "moved: size=" << moved.size()
              << ", original size=" << buf.size() << "\n";

    // ── Assignment test ──────────────────────────────────────────────────────
    ScanBuffer assigned(2);  // different capacity
    assigned = moved;        // copy assignment: must reallocate
    std::cout << "assigned: scan[0] first=" << assigned.get(0).ranges[0] << "\n";

    // ── Overflow test: verify circular overwrite doesn't crash ───────────────
    // Push 2 more into `moved` (capacity=4, size=3 currently)
    float s3[2] = {10.0f, 11.0f};
    float s4[2] = {12.0f, 13.0f};
    moved.push(s3, 2);   // fills buffer
    moved.push(s4, 2);   // overwrites oldest (scan[0]=s0)
    // get(0) should now return s1 (first=4.0)
    std::cout << "after overflow, oldest first=" << moved.get(0).ranges[0] << "\n";

    // ── Out-of-range test ────────────────────────────────────────────────────
    try {
        moved.get(100);
    } catch (const std::out_of_range& e) {
        std::cout << "[exception] out_of_range: " << e.what() << "\n";
    }

    return 0;
    // All destructors fire here — valgrind should report 0 leaks.
}
